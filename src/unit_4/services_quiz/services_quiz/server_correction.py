import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn
from geometry_msgs.msg import Twist
import time


class QuizService(Node):

    def __init__(self):

        super().__init__('service_moving')

        self.srv = self.create_service(
            Turn, 'turn', self.srv_callback)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()

    def srv_callback(self, request, response):

        if request.direction == "right":
            self.twist.angular.z = -request.angular_velocity
        else:
            self.twist.angular.z = request.angular_velocity

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)

        while (i <= request.time):

            self.publisher_.publish(self.twist)
            i += 0.1
            time.sleep(0.1)

        # set velocity to zero to stop the robot
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)

        self.get_logger().info("Turned robot " + request.direction +
                               " for " + str(request.time) + " seconds")

        response.success = True

        return response


def main(args=None):

    rclpy.init(args=args)

    quiz_service = QuizService()

    rclpy.spin(quiz_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
