# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
import rclpy
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile


# from std_srvs.srv import Empty
from custom_interface.srv import FindWall
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Service(Node):
    def __init__(self):
        super().__init__('find_wall_service')
        self.group = ReentrantCallbackGroup()
        self.srv = self.create_service(
            FindWall, 'find_wall', self.service_callback, callback_group=self.group)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # The QoS Profile below is the most used to read LaserScan data and some sensor data
        self.subscriber_ = self.create_subscription(
            LaserScan,
            '/scan',
            self.read_measurement,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=self.group)
        self.timer = 0.5
        self.cmd = Twist()
        self.curr_meas = []
        self.closest_meas_ind = 0
        self.closest_meas_val = 0.0
        self.front_laser_meas = 0.0
        # front laser acceptable range with ± 5 degree of error
        self.acceptable_front_range = range(349, 361)
        # right laser acceptable range with ± 5 degrees of error
        self.acceptable_right_range = range(169, 181)

    # subscriber callback function
    def read_measurement(self, msg):
        self.curr_meas = msg.ranges
        self.front_laser_meas = msg.ranges[359]
        # self.get_logger().info('read laser measurement')

    # def rotate(self):
    #     self.cmd.linear.x = 0.0
    #     self.cmd.angular.z = -0.2
    #     self.publisher_.publish(self.cmd)
    #     self.get_logger().info("Rotating for "+str(self._seconds_sleeping)+" seconds")
    #     for i in range(self._seconds_sleeping):
    #         self.get_logger().info("SLEEPING=="+str(i)+" seconds")
    #         time.sleep(1)

    def stop_robot(self):
        self.get_logger().info('STOPPING ROBOT...')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def rotate_left(self):
        self.get_logger().info(
            'closest wall is on the left - rotating counterclockwise to reach the closest wall')
        self.cmd.linear.x = 0.0  # making sure that the robot is not moving forward
        self.cmd.angular.z = 0.1  # rotate counterclockwise to point to the closest wall
        self.publisher_.publish(self.cmd)

    def rotate_right(self):
        self.get_logger().info(
            'closest wall is on the right - rotating clockwise to reach the closest wall')
        self.cmd.linear.x = 0.0  # making sure that the robot is not moving forward
        self.cmd.angular.z = -0.1  # rotate counterclockwise to point to the closest wall
        self.publisher_.publish(self.cmd)

    def move_forward(self):
        self.get_logger().info('MOVING ROBOT FORWARD...')
        self.cmd.linear.x = 0.05
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)

    def update_closest_measurement_val_and_ind(self):
        # updating closest measurement value and index
        self.closest_meas_ind = self.curr_meas.index(min(self.curr_meas))
        self.closest_meas_val = self.curr_meas[self.closest_meas_ind]

    def service_callback(self, request, response):

        self.get_logger().warn(
            f'Current WallFound Bool Value: {response.wallfound}')
        # Finding closest wall by using the closest laser measurement
        self.update_closest_measurement_val_and_ind()
        self.get_logger().info(
            f'The closest measurement is: {self.closest_meas_val}m and the index is: {self.closest_meas_ind}')

        # rotating the robot until the front laser ray is the smallest
        if self.closest_meas_ind < 359:  # if the closest index is located on the right hand side
            self.rotate_right()
        else:  # the closest index is located on the left hand side
            self.rotate_left()
        self.get_logger().warn(
            f'Value of closest index before while loop: {self.closest_meas_ind}')
        while self.closest_meas_ind not in self.acceptable_front_range:
            self.get_logger().info(
                'Front measurement not in acceptable range')
            # updating the closest measurement index and value
            self.update_closest_measurement_val_and_ind()
            self.get_logger().info(
                f'Update complete - minimum ind ={self.closest_meas_ind} and closest_meas_val = {self.closest_meas_val}m')
        self.get_logger().info('Robot currently facing closest wall - stopping robot...')
        self.stop_robot()

        # Moving the robot forward until the front laser ray has a distance smaller than 30 cm
        self.move_forward()
        while True:
            self.get_logger().info(
                f'Current front laser measurment: {self.front_laser_meas}m')
            if self.front_laser_meas < 0.3:
                self.get_logger().info('Distance to closest wall < 30cm')
                self.stop_robot()
                break

        # Rotating robot until the closest wall is on the right
        self.get_logger().info('Robot can now rotate to have the wall on the right')
        self.rotate_left()
        while True:
            # updating the closest measurement index and value
            self.update_closest_measurement_val_and_ind()
            self.get_logger().info(
                f'Closest measurement index = {self.closest_meas_ind}')
            if self.closest_meas_ind in self.acceptable_right_range:
                self.get_logger().info('The closest wall is currently on the right - Stopping robot...')
                self.stop_robot()
                break

        # At this point, the robot is ready to start following the wall
        response.wallfound = True
        return response


def main(args=None):  # check the main function of a service node
    rclpy.init(args=args)
    try:
        srv = Service()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(srv)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            srv.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main':
    main()
