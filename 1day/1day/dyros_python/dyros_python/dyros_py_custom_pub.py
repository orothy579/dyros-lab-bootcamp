import rclpy
from rclpy.node import Node

from dyros_interface.msg import Sphere


class DyrosPyCustomPublisher(Node):

    def __init__(self, node_name, topic_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Sphere, topic_name, 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.0

    def timer_callback(self):
        msg = Sphere()
        msg.radius =  self.i
        msg.center.x = 0.0
        msg.center.y = 1.0
        msg.center.z = 2.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing:\n\tx: "%.1f"\n\ty: "%.1f\n\tz: "%.1f\n\tr: "%.1f"'
         % (msg.center.x, msg.center.y, msg.center.z, msg.radius))
        self.i += 1.0


def main(args=None):
    rclpy.init(args=args)

    node_name = "dyros_py_pub_node"
    topic_name = "dyros_pub_sub_topic"

    pub = DyrosPyCustomPublisher(node_name, topic_name)

    rclpy.spin(pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
