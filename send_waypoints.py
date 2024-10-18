#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from std_msgs.msg import Header

class WaypointActionClient(Node):
    def __init__(self):
        super().__init__('waypoint_action_client')
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.waypoints = self.create_waypoints()

    def create_waypoints(self):
        # Define waypoints
        return [
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=1.0, y=1.0, z=0.0), orientation=Quaternion(w=1.0))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=2.0, y=2.0, z=0.0), orientation=Quaternion(w=1.0))),
            PoseStamped(header=Header(frame_id='map'), pose=Pose(position=Point(x=3.0, y=3.0, z=0.0), orientation=Quaternion(w=1.0)))
        ]

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = WaypointActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
