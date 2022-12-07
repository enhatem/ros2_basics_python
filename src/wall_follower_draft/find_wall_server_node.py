# FindWall.srv
#
# ---
# bool wallfound



# # function to find minimum and maximum position in list
# def minimum(a, n):
#
#     # inbuilt function to find the position of minimum
#     minpos = a.index(min(a))
#
#     # inbuilt function to find the position of maximum
#     maxpos = a.index(max(a))
#
#     # printing the position
#     print "The maximum is at position", maxpos + 1
#     print "The minimum is at position", minpos + 1
#
# # driver code
# a = [3, 4, 1, 3, 4, 5]
# minimum(a, len(a))

import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from custom_interface import FindWall
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class Service(Node):
    def __init__(self):
        super().__init__('find_wall_service')
        self.srv = self.create_service(FindWall,'find_wall',self.service_callback)
        self.publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        self.subscriber_ = self.create_subscription(
                LaserScan,
                '/scan',
                self.read_measurement, 
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # is the most used to read LaserScan data and some sensor data.
)
        self.timer = 0.5
        self.cmd = Twist()
        self.curr_meas = LaserScan()
        self.front_meas = Float32()
        self.right_meas = Float32()
        self.closest_meas_ind = int()

    def read_measurement(self, msg):
        self.curr_meas = msg
        self.front_meas = msg.ranges[359]
        self.right_meas = msg.ranges[269]

    def service_callback(self, request, response):
        #
        # identifying the shortest laser ray 
        self.closest_meas_ind = self.curr_meas.ranges.index(min(self.curr_meas.ranges))
        self.get_logger().info(f'The angle to the closest wall is: {self.closest_meas_ind+1}')
        # rotating the robot until the front laser ray is the smallest
        if self.closest_meas_ind < 179: 
            self.cmd.linear.x = 0.0 # making sure that the robot is not moving forward
            self.cmd.angular.z = 0.3 # allows to reach the closest wall faster
        else:
            self.cmd.linear.x = 0.0 # making sure that the robot is not moving forward
            self.cmd.angular.z = - 0.3 # allows to reach the closest wall faster
        while self.closest_meas_ind is not 359 :
            self.get_logger().info(f'Rotating to face closest wall ...') 
            self.publisher_.publish(self.cmd)
        self.get_logger().info(f'Facing closest wall - stopping rotation...')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0 # stopping rotation
        self.publisher_.publish(self.cmd) # publishing message
        # moving the robot forward until the distance is 0.3 m
        if self.front_meas > 0.3:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.0
        while self.front_meas > 0.3: # while the robot is far away from the closest wall, approach the closest wall
            self.get_logger().info('Robot is too far from closest wall - Approaching ...')
            self.publisher_.publish(self.cmd)
        self.get_logger().info('Closest wall reached ! Stopping the robot...')
        self.cmd.linear.x = 0.0 # stopping approach
        self.cmd.angular.z = 0.0 
        self.publisher_.publish(self.cmd) # publishing message
        # rotating the robot until ray 270 is facing the wall (is the smallest)
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.3
        self.get_logger().info('Rotating the robot until the wall is on the right side...')
        while self.closest_meas_ind is not 269: 
            self.publisher_.publish(self.cmd)
        self.get_logger().info('Closest wall is on the right! Stopping robot...')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd) # stopping robot
        # returning the service message as true
        self.get_logger().info('Robot is ready to follow wall.')
        response.wallfound = True
        return response

def main(args=None): # check the main function of a service node
    rclpy.init(args=args)
    srv = Service()
    rclpy.spin(srv)
    srv.destroy_service()
    rclpy.shutdown()

if __name__ == '__main':
    main()