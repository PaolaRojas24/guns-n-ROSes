import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

MAX_LINEAR = 0.12
MAX_ANGULAR = 5.0
FRONT_THRESHOLD = 100.0
CORNER_THRESHOLD = 80.0
DESIRED_WALL_DIST = 120.0  # valor de ps5 deseado (distancia ideal a pared izquierda)
KP = 0.025                 # ganancia proporcional


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.__ps5 = 0.0
        self.__ps6 = 0.0
        self.__ps7 = 0.0

        self.create_subscription(Range, '/ps5', self.__ps5_callback, 1)
        self.create_subscription(Range, '/ps6', self.__ps6_callback, 1)
        self.create_subscription(Range, '/ps7', self.__ps7_callback, 1)

    def __ps5_callback(self, message):
        self.__ps5 = message.range

    def __ps6_callback(self, message):
        self.__ps6 = message.range

    def __ps7_callback(self, message):
        self.__ps7 = message.range
        self.__update_velocity()

    def __update_velocity(self):
        command = Twist()

        if self.__ps7 > FRONT_THRESHOLD or self.__ps6 > CORNER_THRESHOLD:
            # Pared al frente o esquina: gira a la derecha en su lugar
            command.linear.x = 0.0
            command.angular.z = -MAX_ANGULAR
        else:
            # Control proporcional: mantiene la distancia ideal a la pared izquierda
            # error > 0 → demasiado cerca → corrige a la derecha
            # error < 0 → demasiado lejos  → corrige a la izquierda
            error = self.__ps5 - DESIRED_WALL_DIST
            correction = -KP * error * MAX_ANGULAR
            correction = max(-MAX_ANGULAR, min(MAX_ANGULAR, correction))

            command.linear.x = MAX_LINEAR
            command.angular.z = correction

        self.__publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
