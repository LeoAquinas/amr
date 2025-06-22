import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class GoalActionClient(Node):
    def __init__(self):
        super().__init__('goal_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 6.943
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        goal_msg.pose.pose.orientation.z = 0.0

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')

        # Set the callback to get the result when the action finishes
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:
            self.get_logger().info('Goal was canceled')
        elif status == 5:
            self.get_logger().info('Goal succeeded')
        elif status == 6:
            self.get_logger().info('Goal aborted')
        elif status == 7:
            self.get_logger().info('Goal rejected')
        else:
            self.get_logger().info(f'Goal ended with status: {status}')


        # Shutdown after the goal completes
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    client = GoalActionClient()
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()
