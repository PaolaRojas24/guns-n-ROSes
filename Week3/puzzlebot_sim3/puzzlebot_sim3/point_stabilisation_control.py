import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import math
import transforms3d

class PointStabilisationNode(Node):
    def __init__(self):
        super().__init__('PointStabilisation_node')

        #Parametros PID
        self.declare_parameter('kp_lin', 1.0)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.1)

        self.declare_parameter('kp_ang', 2.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.05)

        self.declare_parameter('dist_tolerance', 0.02) # m (tolerancia)
        self.declare_parameter('ang_tolerance',  0.05) # rad
        self.declare_parameter('max_linear',  0.38)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('control_rate', 20.0)

        self._load_params()

        # Estados del robot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Setpoint actual 
        self.goal_x = None
        self.goal_y = None
        self.goal_active = False

        # Acumuladores PID
        self._reset_pid()

        # Pub/SUb
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_next = self.create_publisher(Point, 'next_point', 10)

        self.create_subscription(Point, 'setpoint', self.setpoint_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        # Timer control
        period = 1.0 / self.get_parameter('control_rate').value
        self.create_timer(period, self.control_loop)

        self.get_logger().info('Nodo listo.')

    def _load_params(self):
        self.kp_lin = self.get_parameter('kp_lin').value
        self.ki_lin = self.get_parameter('ki_lin').value
        self.kd_lin = self.get_parameter('kd_lin').value

        self.kp_ang = self.get_parameter('kp_ang').value
        self.ki_ang = self.get_parameter('ki_ang').value
        self.kd_ang = self.get_parameter('kd_ang').value

        self.dist_tol = self.get_parameter('dist_tolerance').value
        self.ang_tol = self.get_parameter('ang_tolerance').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value

    def _reset_pid(self):
        self.err_lin_prev = 0.0
        self.err_ang_prev = 0.0
        self.int_lin = 0.0
        self.max_ang = 0.0
        

    #Callbacks

    def setpoint_cb(self, msg: Point):

        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_active = True
        self._reset_pid()
        self.get_logger().info(f'Nuevo setpoint: ({msg.x:.3f}, {msg.y:.3f})')

    def odom_cb(self, msg: Odometry):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        #(w, x, y, z)
        _, _, self.yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')


    #Loop de control PPID
    def control_loop(self):
        if not self.goal_active:
            return
        
        dt =  1.0 / self.get_parameter('control_rate').value

        # Errores
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        err_lin = math.hypot(dx, dy)

        #Angulo deseado, normalizado a [-π, π]
        desired_yaw = math.atan2(dy, dx)
        err_ang = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw)
            )
        
        # Verifica si llego
        if err_lin < self.dist_tol and abs(err_ang) < self.ang_tol:
            self._stop_robot()
            return
        
        # PID lineal 
        self.int_lin += err_lin * dt
        der_lin       = (err_lin - self.err_lin_prev) / dt
        u_lin = (self.kp_lin * err_lin
               + self.ki_lin * self.int_lin
               + self.kd_lin * der_lin)
        self.err_lin_prev = err_lin

        # PID angular 
        self.int_ang += err_ang * dt
        der_ang       = (err_ang - self.err_ang_prev) / dt
        u_ang = (self.kp_ang * err_ang
               + self.ki_ang * self.int_ang
               + self.kd_ang * der_ang)
        self.err_ang_prev = err_ang

        # Si el ángulo es grande, frena lineal para girar primero
        if abs(err_ang) > 0.3:
            u_lin *= max(0.0, 1.0 - abs(err_ang) / math.pi)

        # --- Saturación ---
        u_lin = float(np.clip(u_lin, 0.0,       self.max_lin))
        u_ang = float(np.clip(u_ang, -self.max_ang, self.max_ang))

        # Publicar cmd_vel 
        cmd = Twist()
        cmd.linear.x  = u_lin
        cmd.angular.z = u_ang
        self.pub_cmd.publish(cmd)

    #
    def _stop_robot(self):
        self.pub_cmd.publish(Twist())   # velocidades en cero
        self.goal_active = False
        self.get_logger().info(
            f'Punto alcanzado: ({self.x:.3f}, {self.y:.3f})'
        )

        #Confirma la posición real a TrajectoryNode

        msg =  Point()
        msg.x = self.x
        msg.y = self.y
        self.pub_next.publish(msg)
        self.get_logger().info('next_point enviado.')



def main(args=None):

    rclpy.init(args=args)

    node = PointStabilisationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()