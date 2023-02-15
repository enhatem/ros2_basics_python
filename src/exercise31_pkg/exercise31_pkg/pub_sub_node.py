import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import Twist interface from geometry_msgs
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile


class PubSub(Node):

    def __init__(self):
        # initializing the node object and giving it the name 'publisher_subscriber'
        super().__init__('publisher_subscriber')  # initializing the node object

        # creating publisher object
        self.publisher_ = self.create_publisher(
            Twist, '/cmd_vel', 10)  # QoSProfile(depth=10, realiability=ReliabilityPolicy.RELIABLE))

        self.subscriber_ = self.create_subscription(
            LaserScan, '/scan', self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        # setting the timer period
        self.timer_period = 0.5

        # creating timer sending 2 parameters (the duration between callbacks, and the timer callback function)
        self.timer = self.create_timer(self.timer_period, self.motion)

        # creating a message variable for the publisher
        self.cmd = Twist()

        # creating a variable to store the forward laser measurement
        self.forward_measurement = 0

    def listener_callback(self, laser_meas):
        # reading front laser measurement
        self.forward_measurement = laser_meas.ranges[359]

    def motion(self):
        # displaying current front laser measurement
        self.get_logger().info(f'distance = {self.forward_measurement} m')

        # motion logic
        if self.forward_measurement > 5:
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        elif self.forward_measurement < 5 and self.forward_measurement >= 0.5:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        # publishing velocity commands via the Twist interface to /cmd_vel topic
        self.publisher_.publish(self.cmd)


def main(args=None):

    # initiating the ROS communication
    rclpy.init(args=args)

    # creating an instance of the publisher_subscriber node
    publisher_subscriber = PubSub()

    # spinning the node until it is stopped (using ctrl+c)
    rclpy.spin(publisher_subscriber)

    # explicitly destroying node
    publisher_subscriber.destroy_node()

    # shutting down ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
