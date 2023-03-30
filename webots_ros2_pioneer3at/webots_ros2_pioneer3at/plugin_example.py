"""A simple dummy plugin that demonstrates the usage of Python plugins."""
from rosgraph_msgs.msg import Clock
import rclpy

class PluginExample:
    # The `init` method is called only once the driver is initialized. You will always get two arguments:
    # - The `webots_node` argument contains a reference on a Supervisor instance.
    # - The `properties` argument is a dictionary created from the XML tags.
    def init(self, webots_node, properties):
        # Unfortunately, we cannot get an instance of the parent ROS node. However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.create_node('plugin_node_example')

        # The robot property allows you to access the standard Webots API.
        # See: https://cyberbotics.com/doc/reference/robot
        self.__robot = webots_node.robot
        self.__node.get_logger().info('  - robot name: ' + str(self.__robot.getName()))
        self.__node.get_logger().info('  - basic timestep: ' + str(int(self.__robot.getBasicTimeStep())))

        # The robot property allows you to access the Supervisor Webots API only if the robot is a Supervisor.
        # The function "self.__robot.getSupervisor()" will return "true" in case the robot is a Supervisor.
        # See: https://cyberbotics.com/doc/reference/supervisor
        self.__node.get_logger().info('  - is supervisor? ' + str(self.__robot.getSupervisor()))

        # The robot property also allows you to access the Driver Webots API in case the robot is based on a Car.
        # See: https://cyberbotics.com/doc/automobile/driver-library

        # Create a simple publisher, subscriber and "Clock" variable.
        self.__node.create_subscription(Clock, 'clock', self.__clock_callback, 1)
        self.__publisher = self.__node.create_publisher(Clock, 'custom_clock', 1)
        self.__clock = Clock()

    def __clock_callback(self, msg):
        self.__clock = msg

    # The `step` method is called at every step.
    def step(self):
        # The self.__node has to be spinned once in order to execute callback functions.
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__publisher.publish(self.__clock)