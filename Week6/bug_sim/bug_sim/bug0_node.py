import math

import numpy as np
import rclpy
import transforms3d
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


GO_TO_GOAL   = 'GO_TO_GOAL'
WALL_FOLLOW  = 'WALL_FOLLOW'
GOAL_REACHED = 'GOAL_REACHED'


class Bug0Node(Node):

    def __init__(self):
        super().__init__('bug0_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('goal_x',             2.0)
        self.declare_parameter('goal_y',             0.0)
        self.declare_parameter('obstacle_threshold', 0.40)  # m
        self.declare_parameter('goal_tolerance',     0.20)  # m
        self.declare_parameter('forward_fov_deg',    40.0)  # °
        self.declare_parameter('side_fov_deg',       30.0)  # °
        self.declare_parameter('wall_dist_target',   0.35)  # m
        self.declare_parameter('linear_speed',       0.12)  # m/s
        self.declare_parameter('angular_speed',      0.60)  # rad/s
        self.declare_parameter('kp_wall',            1.20)  # ganancia P wall-follow

        self._load_params()

        # ── Estado ────────────────────────────────────────────────────────────
        self.state    = GO_TO_GOAL
        self.hit_dist = float('inf')

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.scan_front = float('inf')
        self.scan_right = float('inf')
        self.scan_ready = False

        # Controla si el PID ya tiene el setpoint del goal final publicado
        self._pid_goal_sent = False

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_cmd      = self.create_publisher(Twist, '/cmd_vel',  10)
        self.pub_setpoint = self.create_publisher(Point, 'setpoint',  10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Odometry,  'odom',  self.odom_cb,  10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb,  10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.create_timer(1.0,  self.debug_log)     #  1 Hz

        self.get_logger().info(
            f'Bug0Node listo. Goal: ({self.goal_x:.2f}, {self.goal_y:.2f})  '
            f'tolerance={self.goal_tolerance:.2f} m')

    # ──────────────────────────────────────────────────────────────────────────
    def _load_params(self):
        self.goal_x             = self.get_parameter('goal_x').value
        self.goal_y             = self.get_parameter('goal_y').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.goal_tolerance     = self.get_parameter('goal_tolerance').value
        self.forward_fov_deg    = self.get_parameter('forward_fov_deg').value
        self.side_fov_deg       = self.get_parameter('side_fov_deg').value
        self.wall_dist_target   = self.get_parameter('wall_dist_target').value
        self.linear_speed       = self.get_parameter('linear_speed').value
        self.angular_speed      = self.get_parameter('angular_speed').value
        self.kp_wall            = self.get_parameter('kp_wall').value

    # ──────────────────────────────────────────────────────────────────────────
    @staticmethod
    def _sector_min(ranges, center_idx, half_width, n):
        idxs = [(center_idx + i) % n for i in range(-half_width, half_width + 1)]
        vals = ranges[idxs]
        vals = vals[(vals > 0.01) & np.isfinite(vals)]
        return float(np.min(vals)) if len(vals) > 0 else float('inf')

    def _dist_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def _angle_to_goal(self):
        """Error angular al goal en el marco del robot [-π, π]."""
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        desired = math.atan2(dy, dx)
        return math.atan2(math.sin(desired - self.yaw),
                          math.cos(desired - self.yaw))

    def _cmd(self, linear, angular):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.pub_cmd.publish(t)

    def _send_setpoint(self, x: float, y: float):
        """Publica un setpoint para que el control_node (PID) mueva el robot."""
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = 0.0
        self.pub_setpoint.publish(msg)

    def _cancel_pid(self):
        """
        Cancela el PID enviando la posición actual como setpoint (error = 0)
        y detiene el robot directamente para tomar el control en WALL_FOLLOW.
        """
        self._send_setpoint(self.x, self.y)
        self._cmd(0.0, 0.0)
        self._pid_goal_sent = False

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = transforms3d.euler.quat2euler(
            [q.w, q.x, q.y, q.z], axes='sxyz')

    def scan_cb(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=float)
        n   = len(ranges)
        inc = msg.angle_increment

        half_fwd  = max(1, int(math.radians(self.forward_fov_deg / 2) / inc))
        half_side = max(1, int(math.radians(self.side_fov_deg   / 2) / inc))
        idx_right = int(round(math.radians(-90) / inc)) % n

        self.scan_front = self._sector_min(ranges, 0,         half_fwd,  n)
        self.scan_right = self._sector_min(ranges, idx_right, half_side, n)
        self.scan_ready = True

    # ──────────────────────────────────────────────────────────────────────────
    # Loop principal
    # ──────────────────────────────────────────────────────────────────────────
    def control_loop(self):
        if not self.scan_ready or self.state == GOAL_REACHED:
            return

        dist = self._dist_to_goal()

        # ── Llegada al goal — siempre se verifica, sin bloqueos ───────────────
        if dist < self.goal_tolerance:
            self.state = GOAL_REACHED
            self._cancel_pid()
            self.get_logger().info(
                f'*** GOAL ALCANZADO ***  '
                f'pose=({self.x:.3f}, {self.y:.3f})  dist={dist:.3f} m')
            return

        if self.state == GO_TO_GOAL:
            self._step_go_to_goal(dist)
        else:
            self._step_wall_follow(dist)

    # ── GO TO GOAL ─────────────────────────────────────────────────────────────
    def _step_go_to_goal(self, dist: float):
        # ¿Obstáculo al frente?
        if self.scan_front < self.obstacle_threshold:
            self.hit_dist = dist
            self.state    = WALL_FOLLOW
            self._cancel_pid()          # detiene el PID antes de tomar cmd_vel
            self.get_logger().info(
                f'Obstáculo a {self.scan_front:.2f} m → WALL_FOLLOW '
                f'(hit_dist={dist:.2f} m)')
            return

        # Delegar la navegación al control_node (PID).
        # Solo se publica el setpoint una vez por tramo; el PID lo mantiene
        # activo hasta que lo cancelamos o llegamos al goal.
        if not self._pid_goal_sent:
            self._send_setpoint(self.goal_x, self.goal_y)
            self._pid_goal_sent = True
            self.get_logger().info(
                f'Setpoint enviado al PID: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    # ── WALL FOLLOW ────────────────────────────────────────────────────────────
    def _step_wall_follow(self, dist: float):
        front_free = self.scan_front >= self.obstacle_threshold

        # Condición de salida Bug 0
        if front_free and dist < self.hit_dist:
            self.state = GO_TO_GOAL
            self._pid_goal_sent = False  # fuerza re-envío del setpoint al PID
            self.get_logger().info(
                f'Frente libre, dist={dist:.2f} < hit={self.hit_dist:.2f} '
                f'→ GO_TO_GOAL')
            return

        # Control P para mantener distancia a la pared derecha
        wall_error = self.scan_right - self.wall_dist_target
        angular    = float(np.clip(
            -self.kp_wall * wall_error,
            -self.angular_speed,
             self.angular_speed))

        if not front_free:
            # Frente bloqueado: girar en sitio a la izquierda
            self._cmd(0.0, self.angular_speed)
        else:
            self._cmd(self.linear_speed, angular)

    # ──────────────────────────────────────────────────────────────────────────
    # Debug
    # ──────────────────────────────────────────────────────────────────────────
    def debug_log(self):
        dist = self._dist_to_goal()
        self.get_logger().info(
            f'\n'
            f'  Estado       : {self.state}\n'
            f'  Pose robot   : x={self.x:.3f}  y={self.y:.3f}  '
            f'yaw={math.degrees(self.yaw):.1f}°\n'
            f'  Goal         : x={self.goal_x:.3f}  y={self.goal_y:.3f}\n'
            f'  Dist al goal : {dist:.3f} m  (tolerance={self.goal_tolerance:.2f} m)\n'
            f'  LiDAR frente : {self.scan_front:.3f} m  '
            f'(umbral={self.obstacle_threshold:.2f} m)\n'
            f'  LiDAR derecha: {self.scan_right:.3f} m\n'
            f'  hit_dist     : {self.hit_dist:.3f} m'
        )

    # ──────────────────────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_pid()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
