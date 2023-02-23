import rclpy
# import ROS2 python dependencies
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import numpy as np


class TopicsQuiz(Node):
    # class constructor
    def __init__(self):
        # naming the class constructor (node name)
        super().__init__('topics_quiz_node')
        # creating the publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # creating the subscriber
        self.subscriber_ = self.create_subscription(
            Odometry,
            '/odom',
            self.read_odometry,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # based on the QoS profile - Reliability : RELIABLE
        # define a variable for the timer period
        self.timer_period = 0.5
        # declare a variable for the message to send through the publisher
        self.cmd = Twist()
        # declare a variable to store the received odometry message
        self.odom_msg = Odometry()
        self.X0 = Point()  # coordinates at the initial state
        self.X_curr = Point()  # coordinates at the current state
        self.dist_travelled = 0.0  # variable to store the distance travelled before rotation
        # variable used to store the distance travelled after rotation
        self.dist_travelled_V2 = 0.0
        # variable used to store the position after the 90 deg rotation is performed
        self.X0_V2 = Point()
        # variables to store current roll, pitch and yaw angles
        self.curr_roll = 0.0
        self.curr_pitch = 0.0
        self.curr_yaw = 0.0
        # variable used to store current quaternion array
        self.curr_quaternion = [0.0, 0.0, 0.0, 1.0]
        # Boolean variables used in the code
        self.set_initial_coords = False
        self.rotation = False
        self.set_once_V2 = False

        # create timer and send 2 parameters (the time period and the timer callback function)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):

        # Calculating the distance travelled

        if self.rotation is False:
            self.dist_travelled = np.sqrt(
                (self.X_curr.x-self.X0.x)**2 + (self.X_curr.y-self.X0.y)**2)
            # outputing the readings
            if self.dist_travelled < 3:
                self.get_logger().info(
                    f'Distance travelled: {self.dist_travelled} m')
        else:
            self.dist_travelled_V2 = np.sqrt(
                (self.X_curr.x-self.X0_V2.x)**2 + (self.X_curr.y-self.X0_V2.y)**2)

        self.curr_quaternion = [self.odom_msg.pose.pose.orientation.x,
                                self.odom_msg.pose.pose.orientation.y,
                                self.odom_msg.pose.pose.orientation.z,
                                self.odom_msg.pose.pose.orientation.w]
        # Calculating the Euler angles
        self.euler_from_quaternion()

        if self.dist_travelled < 3 and self.rotation is False:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            # self.get_logger().info(f'I published: {str(self.cmd)}')
        elif self.dist_travelled > 3 and self.curr_yaw < 1.57:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.1
            self.publisher_.publish(self.cmd)
            self.get_logger().info(f'Stop and rotate ! : {str(self.curr_yaw)}')
        elif self.dist_travelled > 3 and self.curr_yaw > 1.57:
            if self.rotation is False:
                self.rotation = True
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info(
                f'Rotation done ! Go Straight again !: {str(self.dist_travelled_V2)}')
            if self.rotation is True and self.dist_travelled_V2 > 4:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.get_logger().info(
                    f'Stop the robot! The distance travelled is: {str(self.dist_travelled_V2)}')

    def read_odometry(self, msg):
        self.odom_msg = msg
        if self.set_initial_coords is False:
            # storing the initial position in self.X
            self.X0 = self.odom_msg.pose.pose.position
            self.get_logger().info(f'Initial position coordinates: {self.X0}')
            self.set_initial_coords = True

        if self.rotation is True and self.set_once_V2 is False:
            self.X0_V2 = self.odom_msg.pose.pose.position
            self.set_once_V2 = True

        # storing current position
        self.X_curr = self.odom_msg.pose.pose.position

    def euler_from_quaternion(self):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Below should be replaced when porting for ROS2 Python tf_conversions is done.
        """
        x = self.curr_quaternion[0]
        y = self.curr_quaternion[1]
        z = self.curr_quaternion[2]
        w = self.curr_quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        self.curr_roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        self.curr_pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        self.curr_yaw = np.arctan2(siny_cosp, cosy_cosp)

        # return roll, pitch, yaw


def main(args=None):
    # initialize the ros communication
    rclpy.init(args=args)
    # declare the node constructor
    quiz_node = TopicsQuiz()
    # spinning node until it is interrupted by user (ctrl+c)
    rclpy.spin(quiz_node)
    # explicitly destroy the node
    quiz_node.destroy_node()
    # shutdown the ros comminication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
