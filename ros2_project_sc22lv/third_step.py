import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class ColourIdentifier(Node):
    def __init__(self):
        super().__init__('colour_identifier')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        # Parameters
        self.sensitivity = 10
        self.min_contour_area = 100  # Minimum area to consider a detection
        self.green_found = False

    def callback(self, data):
        try:
            # Convert ROS image to OpenCV
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # Convert to HSV color space
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Define color ranges (proper red detection with wrap-around)
            # Green
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
            # Red (proper wrap-around handling)
            hsv_red_lower1 = np.array([0, 100, 100])
            hsv_red_upper1 = np.array([self.sensitivity, 255, 255])
            hsv_red_lower2 = np.array([180 - self.sensitivity, 100, 100])
            hsv_red_upper2 = np.array([180, 255, 255])
            
            # Create masks
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            
            red_mask1 = cv2.inRange(hsv_image, hsv_red_lower1, hsv_red_upper1)
            red_mask2 = cv2.inRange(hsv_image, hsv_red_lower2, hsv_red_upper2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            
            # Combine masks (for visualization)
            combined_mask = cv2.bitwise_or(green_mask, red_mask)
            
            # Find contours in the green mask
            contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            self.green_found = False
            
            if len(contours) > 0:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > self.min_contour_area:
                    self.green_found = True
                    
                    # Calculate contour properties
                    M = cv2.moments(largest_contour)
                    if M['m00'] > 0:  # Safety check
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                    
                    # Draw visualization
                    (x, y), radius = cv2.minEnclosingCircle(largest_contour)
                    center = (int(x), int(y))
                    radius = int(radius)
                    
                    cv2.circle(image, center, radius, (255, 255, 0), 2)
                    cv2.putText(image, f"Green: {area:.0f}px", (center[0]-50, center[1]-radius-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                    print(f"Green object detected (Area: {area:.0f} pixels)")

            # Display results
            cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('Camera Feed', image)
            cv2.resizeWindow('Camera Feed', 320, 240)
            
            cv2.namedWindow('Green Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Green Mask', green_mask)
            cv2.resizeWindow('Green Mask', 320, 240)
            
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()
    
    rclpy.init(args=None)
    colour_identifier = ColourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(colour_identifier,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()