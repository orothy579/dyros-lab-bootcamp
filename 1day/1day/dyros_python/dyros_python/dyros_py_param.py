from tkinter.font import names
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

class DyrosPyParam(Node):
    def __init__(self, node_name):
        super().__init__(node_name, allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=False)
        timer_period = 2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter("name", "dyros")
        self.declare_parameters(
            namespace="gain",
            parameters=[('p', 100), ('i', 10), ('d', 1)]
        )

    def timer_callback(self):
        name = self.get_parameter('name').get_parameter_value().string_value
        (p, i, d) = self.get_parameters(['gain.p','gain.i','gain.d'])
        self.get_logger().info('name: %s\n p_gain: %.2f\n i_gain: %.2f\n d_gain: %.2f\n'
                            % (name, p.value, i.value, d.value))

        new_name = Parameter('name', Parameter.Type.STRING, name)
        new_p = Parameter('gain.p', Parameter.Type.INTEGER, p.value)
        new_i = Parameter('gain.i', Parameter.Type.INTEGER, i.value)
        new_d = Parameter('gain.d', Parameter.Type.INTEGER, d.value)

        all_new_param = [new_name, new_p, new_i, new_d]        
        self.set_parameters(all_new_param)

def main():
    node_name = "dyros_py_param_node"

    rclpy.init()
    param = DyrosPyParam(node_name)
    rclpy.spin(param)
    param.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()