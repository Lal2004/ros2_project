import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from math import sin, cos, pi
import threading
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        
        self.sensitivity = 10
        self.blue_found = False
 
        self.tpoint_radius = 200.0
        self.tpoints = [
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
        
        self.current_tpoint = 0
        # change this to be more understandable
        self.send_goal(*self.tpoints[self.current_tpoint])
        
    def send_goal(self, x, y, yaw):
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
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    # change this so that another function is (move to next wave point is not used)
    # def get_result_callback(self, future):
    #     if not blue_found:
    #         self.move_to_next_waypoint()
    #         self.get_logger().info(f'Navigation result: {result}')
    
    def get_result_callback(self, future):
        result = future.result().result
        if not self.blue_found:
            self.current_tpoint = (self.current_tpoint + 1) % len(self.tpoints)
            self.send_goal(*self.tpoints[self.current_tpoint])
    
    
    def feedback_callback(self, feedback_msg):
        current_pose = feedback_msg.feedback.current_pose.pose.position
        targetx, targety, _ = self.tpoints[self.current_tpoint]
        distance = ((current_pose.x - targetx)**2 + (current_pose.y - targety)**2)**0.5
        
        if distance < self.tpoint_radius and not self.blue_found:
            self.get_logger()
            self.drive_circle()
    
    def drive_circle(self):
        twist = Twist()
        # twist.linear.x = 0.2
        twist.angular.z = 0.1  
        
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < 6.28e9 and self.blue_found:
            self.publisher.publish(twist)
            rcply.spin_once(self, timeout_sec=0.1)
        self.stop()
        
    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
    
    def move_to_next_waypoint(self):
        self.current_tpoint = (self.current_tpoint + 1) % len(self.tpoints)
        self.send_goal(*self.tpoints[self.current_tpoint])
    
    def go_to_blue(self, contour):
        """Move toward the blue box based on its contour."""
        (x, y), radius = cv2.minEnclosingCircle(contour)
        center_x = int(x)
        image_center = self.image_width // 2  # Assume self.image_width is set in callback
        
        # If box is too far (small in view), move forward
        if radius < 30:  # Adjust threshold as needed
            twist = Twist()
            twist.linear.x = 0.1  # Forward speed
            self.publisher.publish(twist)
        
        # If box is off-center, rotate to center it
        elif abs(center_x - image_center) > 20:  # 20-pixel tolerance
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.1 if (center_x < image_center) else -0.3  # Turn direction
            self.publisher.publish(twist)
        
        # If box is large enough (close), stop
        else:
            self.stop()
            self.get_logger().info("Reached ~1m from blue box!")
    
    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.image_width = image.shape[1]
        except CvBridgeError as e:
            self.get_logger().error(f'cv bridge error: {e}')
            return
        
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        contours, hierarchy = cv2.findContours(blue_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
        # change the following code to the exact same logic as was given in lab5 code
        # if contours:
        #     largest_contour = max(contours, key=cv2.contourArea)
        #     if cv2.contourArea(largest_contour) > 5000:
        #         self.blue_detected = True
        #         self.stop()
        #         self.get_logger().info('Blue box found. Stopping')
        # cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
        # cv2.imshow('Detection', image)
        # cv2.resizeWindow('Detection', 320, 240)
        # cv2.waitKey(3)
        
        if contours:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > 100:  # Threshold for ~1m distance
                self.blue_found = True
                self.stop()
                self.go_to_blue(c)
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, center, radius, (255, 0, 0), 2)
                
                self.stop()
                self.get_logger().info('Blue box found. Stopping')

        cv2.imshow('Detection', image)
        cv2.waitKey(3)
        
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
    except ROSInterruptException:
        pass
    
    robot.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()