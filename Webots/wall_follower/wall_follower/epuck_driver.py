import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

HALF_DISTANCE_BETWEEN_WHEELS = 0.026
WHEEL_RADIUS = 0.0205


class EpuckDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        self.__ps5 = self.__robot.getDevice('ps5')
        self.__ps6 = self.__robot.getDevice('ps6')
        self.__ps7 = self.__robot.getDevice('ps7')
        self.__ps5.enable(self.__timestep)
        self.__ps6.enable(self.__timestep)
        self.__ps7.enable(self.__timestep)

        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('epuck_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.__ps5_pub = self.__node.create_publisher(Range, '/ps5', 1)
        self.__ps6_pub = self.__node.create_publisher(Range, '/ps6', 1)
        self.__ps7_pub = self.__node.create_publisher(Range, '/ps7', 1)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __make_range_msg(self, value):
        msg = Range()
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = 0.1
        msg.min_range = 0.0
        msg.max_range = 4096.0
        msg.range = float(value)
        return msg

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.__ps5_pub.publish(self.__make_range_msg(self.__ps5.getValue()))
        self.__ps6_pub.publish(self.__make_range_msg(self.__ps6.getValue()))
        self.__ps7_pub.publish(self.__make_range_msg(self.__ps7.getValue()))

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        left_speed = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_speed = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.__left_motor.setVelocity(left_speed)
        self.__right_motor.setVelocity(right_speed)
