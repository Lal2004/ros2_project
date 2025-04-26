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

        # self.image_width = 640  # Will be updated from first image
        self.sensitivity = 10
        self.blue_found = False
        self.red_found = False
        self.green_found = False
        # self.buffer = 1.0

        # print("blue found:", self.blue_found)
        self.rate = self.create_rate(10)  # 10 Hz
        self.tpoint_radius = 1.0

        # robot will go to random points, but coordinates below are corners of the map with 1m radius between the walls
        self.tpoints = [
            (-9.5, -14.5, 0.0),
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
        self.rotation_in_progress = False
        self.rotation_end_time = self.get_clock().now()  # Initialize with current time
        self.rotation_direction = 0
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
        twist.linear.x = 0.0  
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
                    cv2.putText(image, f"Blue: {area:.1f}px", (10, 30), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

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
                    cv2.putText(image, f"Red: {area:.1f}px", (10, 60), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

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
                    cv2.putText(image, f"Green: {area:.1f}px", (10, 90), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            # Display images like in the example
            cv2.namedWindow('camera_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('camera_Feed', image)
            cv2.resizeWindow('camera_Feed', 320, 240)
            cv2.waitKey(3)

            cv2.namedWindow('threshold_Feed', cv2.WINDOW_NORMAL)
            cv2.imshow('threshold_Feed', combined_mask)
            cv2.resizeWindow('threshold_Feed', 320, 240)
            cv2.waitKey(3)

            cv2.namedWindow('threshold_Feed2', cv2.WINDOW_NORMAL)
            cv2.imshow('threshold_Feed2', final_image)
            cv2.resizeWindow('threshold_Feed2', 320, 240)
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