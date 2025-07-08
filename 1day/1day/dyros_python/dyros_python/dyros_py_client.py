from dyros_interface.srv import AddThreeInts                            
import sys
import rclpy
from rclpy.node import Node


class DyrosPyClient(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   

    def send_request(self, a, b, c):
        self.req.a,  self.req.b, self.req.c = a, b, c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    node_name = "dyros_py_client_node"

    client = DyrosPyClient(node_name)
    response = client.send_request(int(sys.argv[1]),
                                   int(sys.argv[2]),
                                   int(sys.argv[3]))

    client.get_logger().info(
        'Result of add_three_ints: for %d + %d + %d = %d' %                                
        (client.req.a, client.req.b, client.req.c, response.sum))  

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()