import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from services_quiz_srv.srv import Turn
# from builtin_interfaces.msg import Time
# from std_msgs.msg import Int32


class Service(Node):
    def __init__(self):
        super().__init__('quiz_service_node')
        self.srv = self.create_service(Turn, 'turn', self.service_callback)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.msg = Twist()
        self.t_current = 0
        self.t_final = 0

    def service_callback(self, request, response):
        self.t_current = self.get_clock().now().to_msg().sec
        self.t_final = self.t_current + request.time
        while self.get_clock().now().to_msg().sec < self.t_final:
            if request.direction == "right":
                self.msg.linear.x = 0.0
                self.msg.angular.z = - request.angular_velocity
                self.publisher_.publish(self.msg)
                self.get_logger().info('Rotating to the right')
                response.success = True
            elif request.direction == "left":
                self.msg.linear.x = 0.0
                self.msg.angular.z = request.angular_velocity
                self.publisher_.publish(self.msg)
                self.get_logger().info('Rotating to the left')
                response.success = True
            else:
                self.get_logger().info('Invalid request')
                response.success = False
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.publisher_.publish(self.msg)
        self.get_logger().info("Time is up.. Stopping robot")
        return response


def main(args=None):
    rclpy.init(args=args)
    service_ = Service()
    rclpy.spin(service_)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
