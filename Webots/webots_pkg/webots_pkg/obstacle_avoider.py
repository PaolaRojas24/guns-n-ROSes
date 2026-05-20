import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        #a publisher to publish on cmd_vel
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        #two subscribers each for a sensor
        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 1)

    def __left_sensor_callback(self, message):
        self.__left_sensor_value = message.range

    def __right_sensor_callback(self, message):
        self.__right_sensor_value = message.range

        command_message = Twist()

        #constant velocity
        command_message.linear.x = 0.1

        #obstacle avoidance algorithm
        if self.__left_sensor_value < 0.9 * MAX_RANGE or self.__right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0

        self.__publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()