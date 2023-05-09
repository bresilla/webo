import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import message_filters
import threading
import queue
import numpy as np
import webots_ros2_pioneer3at.path_server.utils as utils

import rclpy
from handy_msgs.action import Nav
from sensor_msgs.msg import NavSatFix

class GetThePosition(Node):
    def __init__(self, queue):
        super().__init__('get_the_position')
        self.pub_ = self.create_publisher(Pose, '/pose/local', 10)
        self.pose_sub = message_filters.Subscriber(self, Odometry, '/odometry/global')
        self.fix_sub = message_filters.Subscriber(self, NavSatFix, '/gps')
        self.pose_sub = message_filters.ApproximateTimeSynchronizer([self.pose_sub, self.fix_sub], 10, slop=10)
        self.pose_sub.registerCallback(self.pose_callback)
        self.queue = queue
        self.pose = Pose()
        self.fix = NavSatFix()

    def pose_callback(self, pose_sub, fix_sub):
        self.pose.position = pose_sub.pose.pose.position
        self.pose.orientation = pose_sub.pose.pose.orientation
        self.fix = fix_sub
        self.pub_.publish(self.pose)
        self.queue.put((self.pose, self.fix))


class GoToPosition(Node):
    def __init__(self, queue):
        super().__init__('go_to_position')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_pose_ = Point()
        self.target_pose_ = Point()
        self.current_orientation_ = Quaternion()
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self.queue = queue

        self._action_server = ActionServer(self, Nav, '/navigation', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        # Shutdown after receiving a result

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Nav.Feedback()
        points = goal_handle.request.initial_path.poses
        new_points = []
        for i in points:
            new_points.append((i.pose.position.x, i.pose.position.y))
        for e, i in enumerate(points):
            target = i.pose.position
            self.target_pose_ = target
            print(f"going to: {i}")
            feedback_msg.wp_reached = e
            while True:
                try:
                    data = self.queue.get(timeout=0.1)
                    pose, fix = data
                    if goal_handle.is_cancel_requested:
                        self.stop_moving()
                        return Nav.Result()
                    if not goal_handle.is_active:
                        self.stop_moving()
                        return Nav.Result()
                    self.current_pose_ = pose.position
                    self.current_orientation_ = pose.orientation
                    twist = Twist()
                    distance, twist.linear.x, twist.angular.z  = self.get_nav_params()
                    self.publisher_.publish(twist)
                    feedback_msg.longitude = fix.longitude
                    feedback_msg.latitude = fix.latitude
                    goal_handle.publish_feedback(feedback_msg)
                    if distance < 0.2: 
                        break
                except queue.Empty:
                    self.get_logger().info('Empty queue')
                    pass
        self.stop_moving()
        goal_handle.succeed()
        result = Nav.Result()
        return result
     
    async def execute_callback2(self, goal_handle):
        while True:
            try:
                data = self.queue.get(timeout=0.1)
                pose, fix = data
                self.get_logger().info('Executing goal...')
                feedback_msg = Nav.Feedback()
                points = goal_handle.request.initial_path.poses
                new_points = []
                for i in points:
                    new_points.append((i.pose.position.x, i.pose.position.y))
                print(utils.local_to_gps_array((fix.latitude, fix.longitude), new_points))
                for e, i in enumerate(points):
                    target = i.pose.position
                    self.target_pose_ = target
                    print(f"going to: {i}")
                    feedback_msg.wp_reached = e
                    while True:
                        if goal_handle.is_cancel_requested:
                            self.stop_moving()
                            return Nav.Result()
                        if not goal_handle.is_active:
                            self.stop_moving()
                            return Nav.Result()
                        self.current_pose_ = pose.position
                        self.current_orientation_ = pose.orientation
                        twist = Twist()
                        distance, twist.linear.x, twist.angular.z  = self.get_nav_params()
                        self.publisher_.publish(twist)
                        feedback_msg.longitude = fix.longitude
                        feedback_msg.latitude = fix.latitude
                        goal_handle.publish_feedback(feedback_msg)
                        if distance < 0.5: 
                            break
                self.stop_moving()
                goal_handle.succeed()
                result = Nav.Result()
                return result
            except queue.Empty:
                self.get_logger().info('Empty queue')
                pass

    def stop_moving(self):
        self.get_logger().info('Stopping...')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def get_nav_params(self, angle_max=0.5, velocity_max=0.1):
        """
        Calculate the distance, velocity, and angular error needed to move from the current position and orientation to the target position and orientation.

        Args:
            angle_max (float): The maximum allowable angular error in radians. Default value is 0.35 radians (approximately 20 degrees).
            velocity_max (float): The maximum allowable velocity in m/s. Default value is 0.15 m/s.

        Returns:
            A tuple containing the distance, velocity, and angular error required to move to the target position and orientation.
            - distance (float): The Euclidean distance between the current position and the target position in meters.
            - velocity (float): The desired velocity in m/s based on the distance between the current position and the target position.
            - angular (float): The desired angular error in radians based on the difference between the current orientation and the desired orientation.
        """
        distance = math.sqrt((self.target_pose_.x - self.current_pose_.x) ** 2 + (self.target_pose_.y - self.current_pose_.y) ** 2)
        # calculate the desired velocity based on the distance to the target position
        velocity = 0.2 * distance
        # calculate the initial heading towards the target position
        preheading = math.atan2(self.target_pose_.y - self.current_pose_.y, self.target_pose_.x - self.current_pose_.x)
        # calculate the current orientation of the robot using quaternions
        orientation = yaw = math.atan2(2 * (self.current_orientation_.w * self.current_orientation_.z + self.current_orientation_.x * self.current_orientation_.y), 
                                       1 - 2 * (self.current_orientation_.y**2 + self.current_orientation_.z**2))
        # calculate the difference between the initial heading and the robot's orientation
        heading = preheading - orientation
        # correct the heading to ensure the robot turns the shortest distance towards the target
        heading_corrected = np.arctan2(np.sin(heading), np.cos(heading))
        # limit the angular error and velocity to the maximum allowable values
        angular = max(-angle_max, min(heading_corrected, angle_max))
        velocity = max(-velocity_max, min(velocity, velocity_max))
        return distance, velocity, angular



def main(args=None):
    rclpy.init(args=args)
    thequeue = queue.Queue()
    try:
        getpos=GetThePosition(thequeue)
        gotopos = GoToPosition(thequeue)
        gotopos.log_file = False
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(getpos)
        executor.add_node(gotopos)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            getpos.destroy_node()
            gotopos.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':\
    main()