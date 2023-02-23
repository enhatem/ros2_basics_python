import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from actions_quiz_msg.action import Distance


class QuizActionClient(Node):
    def __init__(self):
        super().__init__('quiz_action_client')
        self._action_client = ActionClient(self, Distance, 'distance_as')

    def send_goal(self, seconds):
        goal_msg = Distance.Goal()
        goal_msg.seconds = seconds

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
            self.get_logger().info('Goal REJECTED')
            return

        self.get_logger().info('Goal ACCEPTED')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # future.result().result would fetch the equivalent of Distance.Result()
        result = future.result().result
        self.get_logger().info(f'Status = {result.status}')
        self.get_logger().info(
            f'Total distance travelled: {result.total_dist}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        # feedback_msg is not of type Future() nor ClientGoalHandle()
        # feedback_msg is the equivalent of Move.FeedbackMessage()
        # feedback_msg.feedback is equivalent to Move.Feedback()
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_dist}')


def main(args=None):
    rclpy.init(args=args)
    quiz_action_client = QuizActionClient()
    quiz_action_client.send_goal(20)
    rclpy.spin(quiz_action_client)


if __name__ == '__main__':
    main()
