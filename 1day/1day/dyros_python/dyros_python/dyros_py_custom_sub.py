import rclpy
from rclpy.node import Node

from dyros_interface.msg import Sphere


class DyrosPyCustomSubscriber(Node):

    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            Sphere,
            topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Subscribing:\n\tx: "%.1f"\n\ty: "%.1f\n\tz: "%.1f\n\tr: "%.1f"'
         % (msg.center.x, msg.center.y, msg.center.z, msg.radius))


def main(args=None):
    rclpy.init(args=args)

    node_name = "dyros_py_sub_node"
    topic_name = "dyros_pub_sub_topic"
    sub = DyrosPyCustomSubscriber(node_name, topic_name)

    rclpy.spin(sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
