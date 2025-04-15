import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class ColorFollower(Node):
    def __init__(self):
        super().__init__('color_follower')
        self.bridge = CvBridge()
        
        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.callback, 
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        
        # Parameters
        self.sensitivity = 10
        self.too_close_threshold = 30000
        self.min_detection_area = 100
        
        # Flags
        self.green_found = False
        self.blue_found = False
        self.too_close = False

    def callback(self, data):
        try:
            # Convert ROS image to OpenCV
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Color ranges
            # Green
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
            
            # Blue (for stopping)
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
            
            # Create masks
            green_mask = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
            blue_mask = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
            
            # Find contours
            green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Reset flags
            self.green_found = False
            self.blue_found = len(blue_contours) > 0
            
            # Process green detection
            if len(green_contours) > 0:
                c = max(green_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if area > self.min_detection_area:
                    self.green_found = True
                    
                    # Draw visualization
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(image, center, radius, (255, 255, 0), 2)
                    cv2.putText(image, f"Area: {area:.0f}", (center[0]-30, center[1]-radius-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                    self.too_close = area > self.too_close_threshold
            
            # Display
            cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Detection', image)
            cv2.resizeWindow('Detection', 320, 240)
            cv2.waitKey(3)
            
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")

    def walk_forward(self):
        twist = Twist()
        twist.linear.x = 0.2
        self.publisher.publish(twist)

    def walk_backward(self):
        twist = Twist()
        twist.linear.x = -0.2
        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher.publish(twist)

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    robot = ColorFollower()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.blue_found:
                robot.stop()
                print("Blue detected - stopped")
            elif robot.green_found:
                if robot.too_close:
                    robot.walk_backward()
                    print("Too close - moving back")
                else:
                    robot.walk_forward()
                    print("Following green")
            else:
                robot.stop()
    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()