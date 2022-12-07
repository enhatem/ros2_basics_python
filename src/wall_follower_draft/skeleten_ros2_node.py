# importing python ros client for ros2
import rclpy 
from rclpy.node import Node
from std_msgs.msg import Point

def pubsubNode (Node):
    # naming the node
    super().__init__('pub_sub_Node')
    self.publisher_ = self.create_publisher(Point,'/cmd_vel',10)
    self.timer_period = 0.5
    self.timer = self.create_timer(timer_period, callbackFunction)
    self.message_holder = Point()

    def callbackFunction(self):
        # storing the messages to be published in the message variable to be published
        self.message_holder.x = 5.0
        self.message_holder.y = 1.0
        self.message_holder.z = 0.0
        # publishing message
        self.publisher_.publish(message_holder)

def main(args=None):
    rclpy.init(args=args)
    pubsub = pubsubNode()
    rclpy.spin(pubsub)
    pubsub.destroy_node()
    rclpy.shutdown()

if __name__ = '__main__':
    main()