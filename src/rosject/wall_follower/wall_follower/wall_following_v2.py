import sys
import time
from threading import Thread

# import python ros client
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import Twist module from geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from custom_interface.srv import FindWall
from std_srvs.srv import Empty
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor


class Wall_Following(Node):

    def __init__(self):
        # naming the class constructor (node name)
        super().__init__('wall_following')

        self.group1 = ReentrantCallbackGroup()
        self.group2 = ReentrantCallbackGroup()
        self.group3 = ReentrantCallbackGroup()

        # creating client for the /find_wall service server
        self.cli = self.create_client(
            FindWall, 'find_wall', callback_group=self.group1)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FindWall.Request()
        self.future = FindWall.Response()

        # creating the publisher object
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # creating the subscriber object
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.read_measurement,
            # is the most used to read LaserScan data and some sensor data.
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group2
        )
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # declare variable to store all the laser messages
        self.laser_msg = LaserScan()
        # declare the variable to save the front laser scan reading
        self.forward_laser = 0.0
        # declare the variable to save the right laser scan reading
        self.right_laser = 0.0
        # declare the velocity twist message
        self.cmd = Twist()
        # create time
        self.timer = self.create_timer(
            self.timer_period, self.motion, callback_group=self.group3)
        # create Boolean for synchronous request
        self.request_sent = False

    def stop_robot(self):
        self.get_logger().info('STOPPING ROBOT...')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def rotate_left(self):
        self.get_logger().info(
            f'Moving away from right wall - distance to right wall = {self.right_laser} m')
        self.cmd.linear.x = 0.1  # making sure that the robot is not moving forward
        self.cmd.angular.z = 0.2  # rotate counterclockwise to point to the closest wall
        self.publisher_.publish(self.cmd)

    def rotate_right(self):
        self.get_logger().info(
            f'Moving away from right wall - distance to right wall = {self.right_laser} m')
        self.cmd.linear.x = 0.1  # making sure that the robot is not moving forward
        self.cmd.angular.z = -0.2  # rotate counterclockwise to point to the closest wall
        self.publisher_.publish(self.cmd)

    def move_forward(self):
        self.get_logger().info('MOVING ROBOT FORWARD...')
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def hard_left_rotation(self):
        self.get_logger().info(
            f'Getting close to front wall - changing direction - distance to front wall = {self.forward_laser} m')
        self.cmd.linear.x = 0.05
        self.cmd.angular.z = 0.4
        self.publisher_.publish(self.cmd)

    def send_request(self):
        self.get_logger().info('SENDING REQUEST...')
        self.future = self.cli.call(self.req)

    def wall_following(self):

        self.get_logger().info('ENTERING WALL_FOLLOWING FUNCTION...')
        # check if the robot is far from the front wall
        if self.forward_laser > 0.5:
            # if the robot is far from the front wall and far from the right wall, move closer to the right wall
            if self.right_laser > 0.3:
                self.rotate_right()
            # if the robot is far from the front wall and close to the right wall, move away from the right wall
            elif self.right_laser < 0.3:
                self.rotate_left()
            # if the robot is far from the front wall is at a distance between 0.2 m and 0.3 m from the right wall, keep moving forward
            else:
                self.move_forward()
            # if the robot is close to a front wall, increase angular velocity while still going forward
        else:
            self.hard_left_rotation()

        # self.get_logger().info(
        #     f'Front laser measurement : {self.forward_laser} m')
        # self.get_logger().info(
        #     f'Right laser measurement : {self.right_laser} m')

    def motion(self):

        # sending synchronous request
        if self.request_sent is False:
            # making sure that the request will not be called again in the callback function
            self.request_sent = True
            self.send_request()

        if self.future.wallfound is False:
            self.get_logger().info('Service response not received yet')
            time.sleep(1)
        else:
            self.wall_following()

    def read_measurement(self, msg):
        self.laser_msg = msg
        self.forward_laser = self.laser_msg.ranges[359]
        self.right_laser = self.laser_msg.ranges[179]


# def main(args=None):
#     # initialize the ROS communication
#     rclpy.init(args=args)
#     # declare the node constructor
#     wall_following = Wall_Following()

#     spin_thread = Thread(target=rclpy.spin, args=(wall_following,))
#     spin_thread.start()

#     wall_following.send_request()

#     wall_following.destroy_node()
#     # shutdown the ROS communication
#     rclpy.shutdown()

def main(args=None):  # check the main function of a service node
    rclpy.init(args=args)
    try:
        wall_following = Wall_Following()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(wall_following)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            wall_following.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
