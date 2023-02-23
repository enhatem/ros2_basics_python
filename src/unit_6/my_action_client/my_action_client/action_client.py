import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# from t3_action_msg.action import Move
from custom_interfaces.action import Move


class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, Move, 'turtlebot3_as_2')

    def send_goal(self, seconds):
        goal_msg = Move.Goal()
        goal_msg.secs = seconds

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # future.result() is of type ClientGoalHandle(), which is a type that is used to work with Action Clients.
        # It allows us to check if a goal message has been accepted by a server
        # and it allows us to access all the parts of an Action message after it has been modified by the action server
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # future.result().result would fetch the equivalent of Move.Result()
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # feedback_msg does not use future.
        # It is just the way ROS2 is designed to handle this data
        # feedback_msg is the equivalent of Move.FeedbackMessage()
        # feedback_msg.feedback is equivalent to Move.Feedback()
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.send_goal(5)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
