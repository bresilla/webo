from action_msgs.msg import GoalStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from handy_msgs.action import Nav
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class MinimalActionClient(Node):
    def __init__(self):
        super().__init__('nav_instructor')
        self._action_client = ActionClient(self, Nav, '/navigation')
        self.at_point = None


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self.at_point = feedback.feedback.wp_reached if self.at_point is None else self.at_point
        if feedback.feedback.wp_reached != self.at_point:
            self.at_point = feedback.feedback.wp_reached
            self.get_logger().info(f'Point >> {feedback.feedback.wp_reached} << reached')
        self.get_logger().info(f'Latitude: {feedback.feedback.latitude}, Longitude: {feedback.feedback.longitude}')

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result.plan_result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))
        rclpy.shutdown()

    def send_goal(self, pose_array):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        goal_msg = Nav.Goal()

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Add poses from the pose array
        for pose_data in pose_array:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = pose_data[0]
            pose.pose.position.y = pose_data[1]
            pose.pose.position.z = pose_data[2]
            pose.pose.orientation.x = pose_data[3]
            pose.pose.orientation.y = pose_data[4]
            pose.pose.orientation.z = pose_data[5]
            pose.pose.orientation.w = pose_data[6]
            path_msg.poses.append(pose)

        goal_msg.initial_path = path_msg
        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


def main(args=None):
    rclpy.init(args=args)
    action_client = MinimalActionClient()
    poses = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [5.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    action_client.send_goal(poses)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()