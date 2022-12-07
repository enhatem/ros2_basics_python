# import python ros client
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import Twist module from geometry_msgs.msg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
# import Float32 modeule from std_msgs.msg
from std_msgs.msg import Float32


class Wall_Following(Node):

    def __init__(self):
        # naming the class constructor (node name)
        super().__init__('wall_following')
        # creating the publisher object
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # creating the subscriber object 
        self.subscriber_ = self.create_subscription(
                LaserScan,
                '/scan',
                self.read_measurement,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  # is the most used to read LaserScan data and some sensor data.
        )
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # declare variable to store all the laser messages
        self.laser_msg = LaserScan()
        # declare the variable to save the front laser scan reading
        self.forward_laser = Float32()
        # declare the variable to save the right laser scan reading
        self.right_laser = Float32()
        # declare the velocity twist message
        self.cmd = Twist()
        # create time
        self.timer = self.create_timer(self.timer_period, self.motion)

    def motion(self):
        # check if the robot is far from the front wall
        if self.forward_laser > 0.5:
            # if the robot is far from the front wall and far from the right wall, move closer to the right wall 
            if self.right_laser > 0.3 : 
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = -0.05
                self.get_logger().info(f'Moving closer to right wall - distance to right wall = {self.right_laser} m')
            # if the robot is far from the front wall and close to the right wall, move away from the right wall 
            elif self.right_laser < 0.2 :
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = 0.05
                self.get_logger().info(f'Moving away from right wall - distance to right wall = {self.right_laser} m')
            # if the robot is far from the front wall is at a distance between 0.2 m and 0.3 m from the right wall, keep moving forward
            else:
                self.cmd.linear.x = 0.1
                self.cmd.angular.z = 0.0
                self.get_logger().info(f'Moving forward - distance to right wall = {self.right_laser} m')
        # if the robot is close to a front wall, increase angular velocity while still going forward
        else:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.2
            self.get_logger().info(f'Getting close to front wall - changing direction - distance to front wall = {self.forward_laser} m')
        
        self.publisher_.publish(self.cmd)
        self.get_logger().info(f'Front laser measurement : {self.forward_laser} m')
        self.get_logger().info(f'Right laser measurement : {self.right_laser} m')

    def read_measurement(self,msg):
        self.laser_msg = msg
        self.forward_laser = self.laser_msg.ranges[359]
        self.right_laser = self.laser_msg.ranges[269]



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_following = Wall_Following()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_following)
    # Explicity destroy the node
    wall_following.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
