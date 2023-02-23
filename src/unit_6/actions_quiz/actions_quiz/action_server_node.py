import time
import rclpy
import numpy as np
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from actions_quiz_msg.action import Distance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
# Import the libraries to use executors and callback groups
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class QuizActionServer(Node):

    def __init__(self):
        super().__init__('quiz_action_server')
        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self._action_server = ActionServer(
            self, Distance, 'distance_as', self.execute_callback, callback_group=self.group1)
        self.subscriber_ = self.create_subscription(
            Odometry, '/odom', self.odometry_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE), callback_group=self.group2)
        self.publisher_ = self.create_publisher(Float64, '/total_distance', 10)
        self.stored_init_meas = False
        # publisher variable used to send message to the /total_distance topic
        self.pub_var = Float64()
        # used to store initial measurement
        self.xy_0 = Point()
        # used to store current measurement
        self.xy_curr = Point()
        # used to store the distance travelled
        self.curr_dist = 0.0
        # bool to be set to True when the execute_callback method is called
        self.execute_goal = False

    def odometry_callback(self, msg):
        # storing the initial measurement before setting the goal
        if self.stored_init_meas is False and self.execute_goal is True:
            self.xy_0 = msg.pose.pose.position
            self.get_logger().warn(f'Initial measurement: {self.xy_0}')
            # needs to be set back to False at the end of the program
            self.stored_init_meas = True
        # storing current measurement
        self.xy_curr = msg.pose.pose.position
        # self.get_logger().info('Odometry measurements received...')

    def execute_callback(self, goal_handle):

        self.get_logger().info('Initiating goal ...')

        self.execute_goal = True
        feedback_msg = Distance.Feedback()

        self.get_logger().warn(
            f'curr_dist value before reset: {self.curr_dist}')
        self.curr_dist = 0.0
        self.get_logger().warn(
            f'curr_dist value after reset: {self.curr_dist}')
        # resetting coordinates
        self.xy_0.x = 0.0
        self.xy_0.y = 0.0
        self.xy_curr.x = 0.0
        self.xy_curr.y = 0.0

        for i in range(goal_handle.request.seconds):

            # calculating current distance travelled
            self.curr_dist = ((self.xy_curr.x-self.xy_0.x) **
                              2 + (self.xy_curr.y-self.xy_0.y)**2)**0.5
            # storing current distance in the feedback part of the action message
            feedback_msg.current_dist = self.curr_dist
            # publishing current distance to the /total_distance topic
            self.pub_var.data = self.curr_dist
            self.publisher_.publish(self.pub_var)
            # publishing current distance as feedback of the Action server
            goal_handle.publish_feedback(feedback_msg)
            # waiting for 1 second
            time.sleep(1.0)

        # updating the goal_handle state to SUCCEED
        goal_handle.succeed()

        # resetting Boolean variables
        self.stored_init_meas = False
        self.execute_goal = False

        # returning the total distance in the result part of the action message
        result = Distance.Result()
        result.status = True
        result.total_dist = self.curr_dist
        self.get_logger().info(
            f'The total distance travelled is: {result.total_dist}')
        return result


def main(args=None):
    rclpy.init(args=args)
    my_quiz_server = QuizActionServer()
    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    # adding node to executor
    executor.add_node(my_quiz_server)
    try:
        # spin the executor
        executor.spin()
    finally:
        executor.shutdown()
        my_quiz_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
