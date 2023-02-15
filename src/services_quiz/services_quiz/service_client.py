import rclpy
from rclpy.node import Node
from services_quiz_srv.srv import Turn


class ClientAsync(Node):
    def __init__(self):
        super().__init__('client_node')
        # creating client that will interact with the service of type Turn and of name "/turn"
        self.client = self.create_client(Turn, 'turn')
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if the service is not available, a message is displayed
            self.get_logger().info('Service not available, waiting again...')
        # creating empty request
        self.req = Turn.Request()

    def send_request(self):
        # preparing the request
        self.req.direction = "right"  # turning right
        self.req.angular_velocity = 0.2  # rad/s
        self.req.time = 10  # during 10 seconds
        # send the request
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initializing ROS communications
    rclpy.init(args=args)
    # declaring the node constructor
    client = ClientAsync()
    # ruen the send_request() method
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running.
                # If the Service has sent a response, the result will be written
                # to a log message.
                client.future.result()
            except Exception as e:
                client.get_logger().info(f'Service call failed: {e}')
            else:
                client.get_logger().info("The robot is moving...")
            break
    client.destroy_node()
    # shutdown ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
