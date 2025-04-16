import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math
from math import sin, cos, pi
import threading
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal

class TargetPoint(Node):
    def __init__(self):
        super().__init__('target_point')
        
        # lab4
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # lab5
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.image_width = 640  # Will be updated from first image
        self.sensitivity = 10
        self.blue_found = False
        self.buffer = 1.0
        self.rotation_in_progress = False
        self.rotation_end_time = self.get_clock().now()  # Initialize with current time
        self.rotation_direction = 0
        print("blue found:", self.blue_found)
        self.rate = self.create_rate(10)  # 10 Hz
        self.tpoint_radius = 1.0
        self.tpoints = [
            (-5.1, -7.6, 0.0),
            (-9.1, -14.2, 0.0),
            (-10.1, -15.2, 0.0),
            (-11.5, 4.43, 0.0),
            (-2.84, 5.12, 0.0),
            (-1.09, 3.83, 0.0),
            (7.14, 5.45, 0.0),
            (7.87, -2.32, 0.0),
            (-1.77, -2.7, 0.0),
            (0.709, -3.65, 0.0),
            (7.67, -4.2, 0.0),
            (8.41, -13.1, 0.0),
            (-0.473, -8.17, 0.0),
            (-4.22, -6.03, 0.0)
               
        ]
        
        self.action = "nav" # "scan" "approach" "stationary" 
        self.current_tpoint = 0
        # change this to be more understandable
        self.send_goal()
        
    def send_goal(self):
        print("sg")
        x, y, yaw = self.tpoints[self.current_tpoint]
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
    def cancel_goal(self):
        print("cg")
        if hasattr(self, 'goal_handle'):
            self.goal_handle.cancel_goal_async()
        
    def goal_response_callback(self, future):
        print("grc")
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        print("grf")
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        if self.action == "nav":
            self.current_tpoint = (self.current_tpoint + 1) % len(self.tpoints)
            self.send_goal()    
    
    
    def feedback_callback(self, feedback_msg):
        # print("fc")
        if self.action == "nav":
            return
        current_pose = feedback_msg.feedback.current_pose.pose.position
        targetx, targety, _ = self.tpoints[self.current_tpoint]
        distance = ((current_pose.x - targetx)**2 + (current_pose.y - targety)**2)**0.5
        
        if distance < self.tpoint_radius:
            self.get_logger().info("reached 1m radius of tpoint. Stopping and spinning")
            self.action = "scan"
            self.drive_circle()

    def drive_circle(self):
        print("circle")
        twist = Twist()
        twist.linear.x = 0.0  # Forward with 0.2 m/s
        twist.angular.z = 0.2  # Spin in place at 0.2 rad/s
        self.publisher.publish(twist)
        
        # Set timeout for scanning (5 seconds)
        self.scan_timer = self.create_timer(15.0, self.scan_timeout)

        # # Calculate the duration for a full 360-degree spin
        # spin_duration = 2 * math.pi / twist.angular.z  # H 31.4 seconds

        # start_time = self.get_clock().now()

        # while rclpy.ok():
        #     while (self.get_clock().now() - start_time).nanoseconds < spin_duration * 1e9:
        #         # if self.blue_found:
        #         #     self.stop()
        #         #     self.publisher.publish(twist)
        #         #     rclpy.spin_once(self, timeout_sec=0.1)
        #         #     break
        #         self.publisher.publish(twist)
        #         rclpy.spin_once(self, timeout_sec=0.1)

    def scan_timeout(self):
        print("st")
        if self.action == "scan":
            self.get_logger().info("No blue box found, moving to next waypoint")
            self.action = "nav"
            self.current_tpoint = (self.current_tpoint + 1) % len(self.tpoints)
            self.send_goal()    
        self.scan_timer.cancel()

    def s2top(self):
        print("stop")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.cancel_goal()
    
    # def go_to_blue(self, contour):
    #     (x, y), radius = cv2.minEnclosingCircle(contour)
    #     center_x = int(x)
    #     image_center = self.image_width // 2  # Assume self.image_width is set in callback
        
    #     # If box is off-center, rotate to center it
    #     if abs(center_x - image_center) > 20:  # 20-pixel tolerance
    #         self.get_logger().info("centering")
    #         twist = Twist()
    #         twist.linear.x = 0.0
    #         twist.angular.z = 0.05 if (center_x < image_center) else -0.05  # Turn direction
    #         # self.stop()
    #         self.publisher.publish(twist)
        
    #     # If box is too far (small in view), move forward
    #     if radius < 250.0:  # Adjust threshold as needed

    #         self.get_logger().info("too far, going forward")
    #         twist = Twist()
    #         twist.linear.x = 0.1  # Forward speed
    #         self.publisher.publish(twist)
    #         # self.walk_forward()
        
    #     # If box is large enough (close), stop
    #     else:
    #         self.stop()
    #         self.get_logger().info("Reached ~1m from blue box!")
    
    def go_to_2blue(self, box_center, box_radius):
        print("gtb")
        self.stop()
        self.cancel_goal()
        print("gtb stop")
        self.action = "approach"
        twist = Twist()
        
        # Center the box in view
        if abs(box_center[0] - self.image_width//2) > 30:
            print("rotate")
            twist.angular.z = -0.2 if (box_center[0] < self.image_width//2) else 0.2
            # self.scan_timer = self.create_timer(15.0, self.scan_timeout)
            # print("stop after 5s")
            self.publisher.publish(twist)
            return
        
        if abs(box_center[0] - self.image_width//2) <= 30:
            print("towards")

            # Move forward if centered
            twist.linear.x = 0.15
            
            # Estimate distance (calibrate these values!)
            distance_estimate = (1.0 / (box_radius/self.image_width)) * 0.4
            
            if distance_estimate <= self.buffer:
                self.stop()
                self.action = "stationary"
                self.get_logger().info("Successfully reached blue box!")
                return
        
        # self.publisher.publish(twist)
        distance_estimate = (1.0 / (box_radius/self.image_width)) * 0.5

        if distance_estimate > self.buffer:  # If >1m away
            print(f"Moving forward (distance: {distance_estimate:.2f}m)")
            twist.linear.x = 0.15
            self.publisher.publish(twist)
        else:
            print("Reached target distance! Stopping")
            self.stop()
            self.action = "stationary"
            

    
    # def callback(self, data):
    #     try:
    #         image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
    #         cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
    #         cv2.imshow('Detection', image)
    #         cv2.resizeWindow('Detection', 320, 240)
    #         cv2.waitKey(3)
            
    #         self.image_width = image.shape[1]
    #         if self.action in ["scan", "approach"]:
    #             hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
    #             hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
    #             Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #             blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
                
    #             contours, hierarchy = cv2.findContours(blue_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE)
                
    #             if contours: 
    #                 c = max(contours, key=cv2.contourArea)
    #                 if cv2.contourArea(c) > 100:  # Threshold for ~1m distance+
    #                     if hasattr(self, 'scan_timer'):
    #                         self.scan_timer.cancel()
                        
    #                     (x, y), radius = cv2.minEnclosingCircle(c)
    #                     center = (int(x), int(y))
    #                     radius = int(radius)
                        
    #                     cv2.circle(image, center, radius, (255, 0, 0), 2) 
                        
    #                     # self.blue_found = True
    #                     # print("blue found:", self.blue_found) 
                        
    #                     # self.stop()
    #                     # self.cancel_goal()
    #                     # self.go_to_blue(c)
    #                     # self.get_logger().info('Blue box found. Stopping')
    #                     # print("blue!")
    
    #                     self.go_to_blue(center, radius)
                        
   
                         
    #     except CvBridgeError as e:
    #         self.get_logger().error(f'cv bridge error: {e}')
    #         return
    
    def callback(self, data):
        try:
            # Convert and display original image
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.image_width = image.shape[1]
            
            # Show original image
            cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
            cv2.imshow('Original', image)
            cv2.resizeWindow('Original', 320, 240)
            cv2.waitKey(3)
            
            # print("action, ", self.action)
            
            if self.action in ["nav", "scan", "approach"]:
                # Convert to HSV
                hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                # Better blue range (as per lab notes)
                blue_lower1 = np.array([100, 100, 100])  # Wider range for blue
                blue_upper1 = np.array([140, 255, 255])
                
                # Create mask
                blue_mask = cv2.inRange(hsv_image, blue_lower1, blue_upper1)
                
                # Show threshold image for debugging
                # cv2.imshow('Blue Mask', blue_mask)
                cv2.namedWindow('Blue Mask', cv2.WINDOW_NORMAL)
                cv2.imshow('Blue Mask', image)
                cv2.resizeWindow('Blue Mask', 320, 240)
                cv2.waitKey(3)
                    
                # Find contours
                contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                if contours:
                    # Get largest contour
                    c = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(c)
                    
                    # Dynamic area threshold (1% of image area)
                    min_area = 0.01 * (image.shape[0] * image.shape[1])
                    
                    if area > min_area:
                        if hasattr(self, 'scan_timer'):
                            self.scan_timer.cancel()
                        
                        # Get circle properties
                        (x, y), radius = cv2.minEnclosingCircle(c)
                        center = (int(x), int(y))
                        radius = int(radius)
                        
                        # Draw on original image
                        cv2.circle(image, center, radius, (255, 0, 0), 2)
                        cv2.putText(image, f"Blue: {area:.1f}px", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
                        
                        # Approach logic
                        self.go_to_blue(center, radius)
                
                # Show processed image
                # cv2.imshow('Processed', image)
                cv2.namedWindow('Processed', cv2.WINDOW_NORMAL)
                cv2.imshow('Processed', image)
                cv2.resizeWindow('Processed', 320, 240)
                cv2.waitKey(3)
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
        
def main(args=None):
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=args)
    robot = TargetPoint()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            continue
    except ROSInterruptException or KeyboardInterrupt:
        pass
    
    robot.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()