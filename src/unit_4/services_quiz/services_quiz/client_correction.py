# link to correction
# https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_ROS2/extra_files/ros2_services_quiz_solution.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=oslFylll6GSHr7NpXDKwtQWizag%3D&Expires=1670194834
import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn


class QuizClient(Node):

    def __init__(self):

        super().__init__('stop_client')

        self.client = self.create_client(Turn, 'turn')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = Turn.Request()

    def send_request(self):

        self.req.direction = 'right'
        self.req.angular_velocity = 0.2
        self.req.time = 10
        self.future = self.client.call_async(self.req)


def main(args=None):

    rclpy.init(args=args)
    client = QuizClient()
    client.send_request()

    while rclpy.ok():

        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response is True:
                    client.get_logger().info(
                        'Robot turned correctly')
                else:
                    client.get_logger().info(
                        'Something went wrong')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
