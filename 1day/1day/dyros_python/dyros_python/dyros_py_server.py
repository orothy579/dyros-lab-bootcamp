from dyros_interface.srv import AddThreeInts                                                           

import rclpy
from rclpy.node import Node


class DyrosPyServer(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)       

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                   
        self.get_logger().info('Incoming request\na: %f b: %f c: %f' % (request.a, request.b, request.c))  

        return response

def main(args=None):
    node_name = "dyros_py_server_node"

    rclpy.init(args=args)

    srv = DyrosPyServer(node_name)

    rclpy.spin(srv)

    rclpy.shutdown()

if __name__ == '__main__':
    main()