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
        
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

        self.sensitivity = 10
        self.blue_found = False
        self.red_found = False
        self.green_found = False

        self.rate = self.create_rate(10)  # 10 Hz
        self.tpoint_radius = 1.0

        # coordinates below are corners of the map with 1m radius between the walls
        # these were hardcoded so that robot can explore the whole map 
        self.tpoints = [
            (-9.5, -14.5, 0.0),
            (2.3, -10.0, 0.0),
            (-11.0, 3.7, 0.0),
            (-3.2, 3.4, 0.0),
            (-1.5, -3.1, 0.0),
            (-1.6, 4.7, 0.0),
            (7.1, 5.5, 0.0),
            (7.9, -2.3, 0.0),
            (0.6, -3.6, 0.0),
            (7.8, -3.9, 0.0),
            (8.5, -13.0, 0.0),
            (-0.3, -8.1, 0.0),
            (-4.4, -5.9, 0.0)
        ]

        
        self.action = "nav" # "scan" "approach" "stationary" 
        self.spin_time = None
        self.spin_start_time = None  # To track when spinning began
        # self.rotation_direction = 0
        self.current_tpoint = 0
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
            # self.current_tpoint = (self.current_tpoint + 1) % len(self.tpoints)
            # self.send_goal()    
            self.spin_for_duration(30)

    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # print(feedback)

    def spin_for_duration(self, duration_sec=30):
        """Spin the robot for specified duration then go to next point"""
        if not hasattr(self, 'spin_timer') or self.spin_timer is None:
            self.cancel_goal()
            # First call - start spinning
            self.get_logger().info(f'Starting to spin for {duration_sec} seconds')
            self.spin_start_time = self.get_clock().now()
            print(self.spin_start_time)
            # Start spinning motion
            twist = Twist()
            twist.linear.x = 0.0  
            twist.angular.z = 0.5 
            self.publisher.publish(twist)
            # self.get_logger().info(f"Published Twist command: linear.x={twist.linear.x}, angular.z={twist.angular.z}")
            
            # Create timer to check duration
            self.spin_timer = self.create_timer(0.1, lambda: self.spin_for_duration(duration_sec))
        else:
            # Subsequent calls - check duration
            time_taken = (self.get_clock().now() - self.spin_start_time).nanoseconds / 1e9
            if time_taken >= duration_sec:
                # Time's up - stop spinning and proceed
                self.stop()
                self.spin_timer.cancel()
                self.spin_timer = None
                self.get_logger().info('Finished spinning')
                # Move to next point
                self.current_tpoint = (self.current_tpoint + 1) % len(self.tpoints)
                self.send_goal()

    def stop(self):
        print("stop")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        # self.cancel_goal()

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            # self.image_width = image.shape[1]

            # Set HSV ranges with sensitivity parameter
            hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
            hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
            hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
            hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
            hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
            hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])

            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Create masks for each color
            blue_image = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)
            red_image = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
            green_image = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)

            # Combine masks (optional)
            combined_mask = cv2.bitwise_or(blue_image, cv2.bitwise_or(red_image, green_image))
            final_image = cv2.bitwise_and(image, image, mask=combined_mask)

            # Process blue
            # self.blue_found = False
            blue_contours, _ = cv2.findContours(blue_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if blue_contours:
                c = max(blue_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 100:  # Using fixed threshold like in the example
                    self.blue_found = True
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    cv2.circle(image, center, int(radius), (255, 0, 0), 2)
                    cv2.putText(image, f"Blue: {area:.1f}", (50, 80), 
                              cv2.FONT_HERSHEY_DUPLEX, 1.2, (255,255,255), 2)

            # Process red (following the example logic)
            # self.red_found = False
            red_contours, _ = cv2.findContours(red_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if red_contours:
                c = max(red_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 100:
                    self.red_found = True
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    cv2.circle(image, center, int(radius), (0, 0, 255), 2)
                    cv2.putText(image, f"Red: {area:.1f}", (50, 120), 
                              cv2.FONT_HERSHEY_DUPLEX, 1.2, (255,255,255), 2)

            # Process green (following the example logic)
            # self.green_found = False
            green_contours, _ = cv2.findContours(green_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if green_contours:
                c = max(green_contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                if area > 100:
                    self.green_found = True
                    (x, y), radius = cv2.minEnclosingCircle(c)
                    center = (int(x), int(y))
                    cv2.circle(image, center, int(radius), (0, 255, 0), 2)
                    cv2.putText(image, f"Green: {area:.1f}", (50, 160), 
                              cv2.FONT_HERSHEY_DUPLEX, 1.2, (255,255,255), 2)

            # Display images like in the example
            cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed', 540, 540)
            cv2.waitKey(3)

            # cv2.namedWindow('threshold_Feed', cv2.WINDOW_NORMAL)
            # cv2.imshow('threshold_Feed', combined_mask)
            # cv2.resizeWindow('threshold_Feed', 540, 540)
            # cv2.waitKey(3)

            cv2.namedWindow('threshold_Feed2', cv2.WINDOW_NORMAL)
            cv2.imshow('threshold_Feed2', final_image)
            cv2.resizeWindow('threshold_Feed2', 540, 540)
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