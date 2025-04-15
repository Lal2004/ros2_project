import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class colourIdentifier(Node):
    def __init__(self):
        super().__init__('cI')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.callback, 
            10)
        self.subscription  # prevent unused variable warning
        self.sensitivity = 10
        
    def callback(self, data):
        try:
            # Convert ROS image to OpenCV
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            
            # Show original image
            cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed', 320, 240)
            
            # Convert to HSV
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Define color ranges (fixed red detection)
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
            
            # Combine masks
            combined_mask = cv2.bitwise_or(green_mask, red_mask)
            
            # Apply mask to original image
            filtered_image = cv2.bitwise_and(image, image, mask=combined_mask)
            
            # Display results
            cv2.namedWindow('Binary Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Binary Mask', combined_mask)
            cv2.resizeWindow('Binary Mask', 320, 240)
            
            cv2.namedWindow('Filtered Image', cv2.WINDOW_NORMAL)
            cv2.imshow('Filtered Image', filtered_image)
            cv2.resizeWindow('Filtered Image', 320, 240)
            
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

def main():
    def signal_handler(sig, frame):
        rclpy.shutdown()
    
    rclpy.init(args=None)
    cI = colourIdentifier()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(cI,), daemon=True)
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