"""Tangent Bug — planificador local reactivo basado en LiDAR."""
import math
import numpy as np
import rclpy
import transforms3d
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


MOTION_TO_GOAL  = 'MOTION_TO_GOAL'
BOUNDARY_FOLLOW = 'BOUNDARY_FOLLOW'

_N_GOALS = 4


class TangentBugNode(Node):

    def __init__(self):
        super().__init__('tangent_bug_node')

        # ── Detección de obstáculos / waypoints ───────────────────────────────
        self.declare_parameter('obstacle_threshold', 0.40)
        self.declare_parameter('goal_tolerance',     0.20)
        self.declare_parameter('forward_fov_deg',    40.0)
        self.declare_parameter('side_fov_deg',       30.0)

        # ── Wall-following (cmd_vel directo) ──────────────────────────────────
        self.declare_parameter('wall_dist_target',   0.35)
        self.declare_parameter('wall_linear_speed',  0.12)
        self.declare_parameter('wall_angular_speed', 0.60)
        self.declare_parameter('kp_wall',            1.20)

        # ── Parámetros propios de Tangent Bug ─────────────────────────────────
        self.declare_parameter('discontinuity_threshold', 0.30)
        self.declare_parameter('sensor_range',            3.5)
        self.declare_parameter('leave_margin',            0.05)
        self.declare_parameter('local_min_tolerance',     0.10)
        self.declare_parameter('local_min_stall_iters',   15)
        self.declare_parameter('bf_grace_iters',          20)
        self.declare_parameter('setpoint_change_thresh',  0.15)
        # Compromiso con el endpoint elegido (evita oscilación al inicio)
        self.declare_parameter('endpoint_reach_thresh',   0.30)
        self.declare_parameter('endpoint_switch_hyst',    0.40)
        # Filtro angular para endpoints: máx desviación respecto al rumbo al goal
        self.declare_parameter('endpoint_max_angle_deg',  120.0)

        for i in range(1, _N_GOALS + 1):
            self.declare_parameter(f'trajectory_goals.goal_{i}.x', 0.0)
            self.declare_parameter(f'trajectory_goals.goal_{i}.y', 0.0)

        self._load_params()

        self.waypoints      = self._load_waypoints()
        self.waypoint_index = 0

        if not self.waypoints:
            self.get_logger().error(
                'No se cargaron waypoints. Verifica el config YAML del mundo.'
            )
        else:
            self.get_logger().info(
                f'Trayectoria cargada ({len(self.waypoints)} waypoints):'
            )
            for i, (wx, wy) in enumerate(self.waypoints):
                self.get_logger().info(f'  goal_{i + 1}: ({wx:.2f}, {wy:.2f})')

        # ── Pose ──────────────────────────────────────────────────────────────
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._odom_ready = False

        # ── LiDAR ─────────────────────────────────────────────────────────────
        self.ranges     = None
        self.angle_min  = 0.0
        self.angle_inc  = 0.0
        self.scan_ready = False

        # ── Estado Tangent Bug ────────────────────────────────────────────────
        self.state             = MOTION_TO_GOAL
        self.h_min             = float('inf')
        self.h_stall_count     = 0
        self.d_followed        = float('inf')
        self.d_reach           = float('inf')
        self.bf_side           = 'right'
        self.bf_iter_count     = 0
        self.committed_target  = None    # (ex, ey) — endpoint comprometido

        # PID setpoint cache
        self._pid_target    = None
        self._pid_goal_sent = False

        # ── Publishers / Subscribers / Timers ─────────────────────────────────
        self.pub_cmd      = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_setpoint = self.create_publisher(Point, 'setpoint', 10)

        self.create_subscription(Odometry,  'odom',  self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0,  self.debug_log)

    # ── Carga de parámetros ───────────────────────────────────────────────────

    def _load_params(self):
        self.obstacle_threshold      = self.get_parameter('obstacle_threshold').value
        self.goal_tolerance          = self.get_parameter('goal_tolerance').value
        self.forward_fov_deg         = self.get_parameter('forward_fov_deg').value
        self.side_fov_deg            = self.get_parameter('side_fov_deg').value
        self.wall_dist_target        = self.get_parameter('wall_dist_target').value
        self.wall_linear_speed       = self.get_parameter('wall_linear_speed').value
        self.wall_angular_speed      = self.get_parameter('wall_angular_speed').value
        self.kp_wall                 = self.get_parameter('kp_wall').value
        self.discontinuity_threshold = self.get_parameter('discontinuity_threshold').value
        self.sensor_range            = self.get_parameter('sensor_range').value
        self.leave_margin            = self.get_parameter('leave_margin').value
        self.local_min_tolerance     = self.get_parameter('local_min_tolerance').value
        self.local_min_stall_iters   = int(self.get_parameter('local_min_stall_iters').value)
        self.bf_grace_iters          = int(self.get_parameter('bf_grace_iters').value)
        self.setpoint_change_thresh  = self.get_parameter('setpoint_change_thresh').value
        self.endpoint_reach_thresh   = self.get_parameter('endpoint_reach_thresh').value
        self.endpoint_switch_hyst    = self.get_parameter('endpoint_switch_hyst').value
        self.endpoint_max_angle_deg  = self.get_parameter('endpoint_max_angle_deg').value

    def _load_waypoints(self):
        waypoints = []
        for i in range(1, _N_GOALS + 1):
            x = self.get_parameter(f'trajectory_goals.goal_{i}.x').value
            y = self.get_parameter(f'trajectory_goals.goal_{i}.y').value
            if x == 0.0 and y == 0.0:
                continue
            waypoints.append((float(x), float(y)))
        return waypoints

    # ── Goal activo ───────────────────────────────────────────────────────────

    @property
    def goal_x(self):
        return self.waypoints[self.waypoint_index][0] if self.waypoints else 0.0

    @property
    def goal_y(self):
        return self.waypoints[self.waypoint_index][1] if self.waypoints else 0.0

    def _advance_to_next_waypoint(self):
        prev_idx = self.waypoint_index
        self.waypoint_index   = (self.waypoint_index + 1) % len(self.waypoints)
        self.state            = MOTION_TO_GOAL
        self.h_min            = float('inf')
        self.h_stall_count    = 0
        self.d_followed       = float('inf')
        self.d_reach          = float('inf')
        self.committed_target = None
        self._pid_goal_sent   = False
        self._pid_target      = None
        self.get_logger().info(
            f'Waypoint {prev_idx + 1} alcanzado → '
            f'siguiente: goal_{self.waypoint_index + 1} '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})'
        )

    # ── Comandos ──────────────────────────────────────────────────────────────

    def _cmd(self, linear, angular):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self.pub_cmd.publish(t)

    def _send_setpoint(self, x, y):
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0
        self.pub_setpoint.publish(msg)

    def _drive_to(self, x, y):
        """Publica setpoint al PID, evitando spam si el objetivo no cambió."""
        if (self._pid_target is None
                or math.hypot(x - self._pid_target[0],
                              y - self._pid_target[1]) > self.setpoint_change_thresh
                or not self._pid_goal_sent):
            self._send_setpoint(x, y)
            self._pid_target    = (x, y)
            self._pid_goal_sent = True
            self.get_logger().info(f'Setpoint → PID: ({x:.2f}, {y:.2f})')

    def _cancel_pid(self):
        self._send_setpoint(self.x, self.y)
        self._cmd(0.0, 0.0)
        self._pid_goal_sent = False
        self._pid_target    = None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = transforms3d.euler.quat2euler(
            [q.w, q.x, q.y, q.z], axes='sxyz')
        self._odom_ready = True

    def scan_cb(self, msg: LaserScan):
        self.ranges     = np.array(msg.ranges, dtype=float)
        self.angle_min  = float(msg.angle_min)
        self.angle_inc  = float(msg.angle_increment)
        self.scan_ready = True

    # ── Geometría LiDAR ───────────────────────────────────────────────────────

    def _angle_of(self, idx, n):
        return self.angle_min + idx * self.angle_inc

    def _idx_of(self, angle, n):
        return int(round((angle - self.angle_min) / self.angle_inc)) % n

    def _polar_to_world(self, r, theta_local):
        return (self.x + r * math.cos(self.yaw + theta_local),
                self.y + r * math.sin(self.yaw + theta_local))

    def _clean_ranges(self):
        r = np.array(self.ranges, dtype=float)
        r[~np.isfinite(r)] = self.sensor_range
        r[r > self.sensor_range] = self.sensor_range
        r[r < 0.01] = self.sensor_range
        return r

    def _sector_min_around_angle(self, target_angle, half_fov_rad):
        """Mínimo del LiDAR alrededor de un ángulo (en frame del robot)."""
        if self.ranges is None or self.angle_inc == 0.0:
            return float('inf')
        r = np.array(self.ranges, dtype=float)
        n = len(r)
        center = self._idx_of(target_angle, n)
        half   = max(1, int(half_fov_rad / self.angle_inc))
        idxs   = [(center + k) % n for k in range(-half, half + 1)]
        vals   = r[idxs]
        vals   = vals[(vals > 0.01) & np.isfinite(vals)]
        return float(np.min(vals)) if len(vals) > 0 else float('inf')

    def _ranges_front(self):
        return self._sector_min_around_angle(0.0, math.radians(self.forward_fov_deg / 2))

    def _ranges_side(self, side):
        ang = -math.pi / 2 if side == 'right' else math.pi / 2
        return self._sector_min_around_angle(ang, math.radians(self.side_fov_deg / 2))

    def _dist_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    # ── Núcleo Tangent Bug ────────────────────────────────────────────────────

    def _is_goal_visible(self):
        """¿El rayo del LiDAR hacia el goal supera la distancia al goal?"""
        if self.ranges is None:
            return False
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        d_goal = math.hypot(dx, dy)
        theta_goal = math.atan2(dy, dx) - self.yaw
        theta_goal = math.atan2(math.sin(theta_goal), math.cos(theta_goal))
        n = len(self.ranges)
        idx = self._idx_of(theta_goal, n)
        half = max(1, int(math.radians(5.0) / self.angle_inc))
        for k in range(-half, half + 1):
            j = (idx + k) % n
            r = self.ranges[j]
            if not math.isfinite(r) or r <= 0.01:
                continue
            if r < d_goal:
                return False
        return True

    def _find_endpoints(self):
        """
        Detecta endpoints O_i (discontinuidades en el LiDAR), filtrando los
        que estén demasiado lejos angularmente del rumbo al goal — esto evita
        que el robot persiga endpoints "detrás" suya al inicio.
        """
        if self.ranges is None or self.angle_inc == 0.0:
            return []
        clean = self._clean_ranges()
        n = len(clean)

        # Ángulo al goal en frame del robot (para filtrar endpoints)
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        theta_goal = math.atan2(dy, dx) - self.yaw
        theta_goal = math.atan2(math.sin(theta_goal), math.cos(theta_goal))
        max_dev = math.radians(self.endpoint_max_angle_deg)

        endpoints = []
        for i in range(n):
            j = (i + 1) % n
            if abs(clean[i] - clean[j]) <= self.discontinuity_threshold:
                continue
            idx = i if clean[i] < clean[j] else j
            r = clean[idx]
            if r >= self.sensor_range - 0.01:
                continue
            theta = self._angle_of(idx, n)
            # Filtra endpoints muy desviados del rumbo al goal
            dev = math.atan2(math.sin(theta - theta_goal),
                             math.cos(theta - theta_goal))
            if abs(dev) > max_dev:
                continue
            ex, ey = self._polar_to_world(r, theta)
            h = r + math.hypot(self.goal_x - ex, self.goal_y - ey)
            endpoints.append(
                {'ex': ex, 'ey': ey, 'r': r, 'theta': theta, 'h': h, 'idx': idx}
            )
        return endpoints

    def _best_endpoint(self):
        eps = self._find_endpoints()
        return min(eps, key=lambda e: e['h']) if eps else None

    def _h_of_point(self, px, py):
        return (math.hypot(px - self.x, py - self.y)
                + math.hypot(self.goal_x - px, self.goal_y - py))

    def _compute_d_reach(self):
        """Distancia mínima al goal entre todos los puntos sensados de la frontera."""
        if self.ranges is None or self.angle_inc == 0.0:
            return float('inf')
        r = np.array(self.ranges, dtype=float)
        n = len(r)
        mask = np.isfinite(r) & (r > 0.01) & (r < self.sensor_range)
        if not np.any(mask):
            return float('inf')
        idxs   = np.where(mask)[0]
        thetas = self.angle_min + idxs * self.angle_inc
        px = self.x + r[idxs] * np.cos(self.yaw + thetas)
        py = self.y + r[idxs] * np.sin(self.yaw + thetas)
        d  = np.hypot(self.goal_x - px, self.goal_y - py)
        return float(np.min(d))

    def _choose_bf_side(self, endpoint):
        """
        Elige lado de seguimiento de pared para rodear el obstáculo.
        Endpoint a la derecha (theta<0) ⇒ rodearemos por la derecha, con
        el obstáculo a nuestra izquierda ⇒ wall follow izquierdo.
        """
        return 'left' if endpoint['theta'] < 0 else 'right'

    # ── Loop principal ────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.scan_ready or not self._odom_ready or not self.waypoints:
            return

        dist = self._dist_to_goal()

        if dist < self.goal_tolerance:
            self._cancel_pid()
            self._advance_to_next_waypoint()
            return

        if self.state == MOTION_TO_GOAL:
            self._step_motion_to_goal(dist)
        else:
            self._step_boundary_follow(dist)

    # ── Motion-to-goal ────────────────────────────────────────────────────────

    def _step_motion_to_goal(self, dist):
        # Goal visible directamente → ir al goal, soltar compromiso
        if self._is_goal_visible():
            if dist < self.h_min:
                self.h_min = dist
            self.h_stall_count    = 0
            self.committed_target = None
            self._drive_to(self.goal_x, self.goal_y)
            return

        best = self._best_endpoint()
        if best is None:
            self.get_logger().warn('Sin endpoints visibles → BOUNDARY_FOLLOW')
            self.bf_side = 'right'
            self._enter_boundary_follow(dist)
            return

        h = best['h']

        # Mínimo local: h(O*) no mejora durante N iteraciones consecutivas
        if h > self.h_min + self.local_min_tolerance:
            self.h_stall_count += 1
        else:
            self.h_stall_count = 0

        if self.h_stall_count > self.local_min_stall_iters:
            self.get_logger().info(
                f'Mínimo local: h(O*)={h:.2f} > h_min={self.h_min:.2f} '
                f'durante {self.h_stall_count} iters → BOUNDARY_FOLLOW'
            )
            self.bf_side = self._choose_bf_side(best)
            self._enter_boundary_follow(dist)
            return

        if h < self.h_min:
            self.h_min = h

        # ── Compromiso con un endpoint para evitar oscilación ────────────
        if self.committed_target is None:
            self.committed_target = (best['ex'], best['ey'])
            self.get_logger().info(
                f'Compromiso con endpoint: ({best["ex"]:.2f}, {best["ey"]:.2f})  '
                f'h={h:.2f}'
            )
        else:
            cx, cy = self.committed_target
            d_to_target = math.hypot(cx - self.x, cy - self.y)

            # Llegamos al endpoint comprometido → renovamos al mejor actual
            if d_to_target < self.endpoint_reach_thresh:
                self.committed_target = (best['ex'], best['ey'])
            else:
                # Solo cambiamos si la nueva opción es CLARAMENTE mejor
                committed_h = self._h_of_point(cx, cy)
                if best['h'] < committed_h - self.endpoint_switch_hyst:
                    self.get_logger().info(
                        f'Cambio de endpoint: h_new={best["h"]:.2f} < '
                        f'h_committed={committed_h:.2f}'
                    )
                    self.committed_target = (best['ex'], best['ey'])

        ex, ey = self.committed_target
        self._drive_to(ex, ey)

    # ── Boundary-following ────────────────────────────────────────────────────

    def _enter_boundary_follow(self, dist):
        self.state            = BOUNDARY_FOLLOW
        self.d_followed       = dist
        self.d_reach          = self._compute_d_reach()
        self.bf_iter_count    = 0
        self.committed_target = None
        self._cancel_pid()
        self.get_logger().info(
            f'BF inicio: lado={self.bf_side}  d_followed={self.d_followed:.2f}  '
            f'd_reach={self.d_reach:.2f}'
        )

    def _step_boundary_follow(self, dist):
        self.bf_iter_count += 1

        if dist < self.d_followed:
            self.d_followed = dist

        d_reach_now = self._compute_d_reach()
        if d_reach_now < self.d_reach:
            self.d_reach = d_reach_now

        # Condición de salida (con gracia para evitar oscilación al entrar)
        if (self.bf_iter_count > self.bf_grace_iters
                and self.d_reach < self.d_followed - self.leave_margin):
            self.get_logger().info(
                f'Salida BF: d_reach={self.d_reach:.2f} < d_followed={self.d_followed:.2f} '
                f'→ MOTION_TO_GOAL'
            )
            self.state            = MOTION_TO_GOAL
            self.h_min            = dist
            self.h_stall_count    = 0
            self.committed_target = None
            self._pid_goal_sent   = False
            self._pid_target      = None
            return

        # Wall following reactivo (cmd_vel directo, sin PID de posición)
        side_dist  = self._ranges_side(self.bf_side)
        front_dist = self._ranges_front()
        front_free = front_dist >= self.obstacle_threshold

        wall_error = side_dist - self.wall_dist_target
        sign       = -1.0 if self.bf_side == 'right' else 1.0
        angular    = float(np.clip(
            sign * self.kp_wall * wall_error,
            -self.wall_angular_speed,
             self.wall_angular_speed))

        if not front_free:
            # Frente bloqueado → gira en sitio alejándose del lado seguido
            self._cmd(0.0, sign * self.wall_angular_speed)
        else:
            self._cmd(self.wall_linear_speed, angular)

    # ── Debug ─────────────────────────────────────────────────────────────────

    def debug_log(self):
        dist  = self._dist_to_goal()
        front = self._ranges_front()
        self.get_logger().info(
            f'\n'
            f'  Estado          : {self.state}\n'
            f'  Waypoint activo : goal_{self.waypoint_index + 1} '
            f'({self.goal_x:.2f}, {self.goal_y:.2f})\n'
            f'  Pose robot      : x={self.x:.3f}  y={self.y:.3f}  '
            f'yaw={math.degrees(self.yaw):.1f}°\n'
            f'  Dist al goal    : {dist:.3f} m  (tol={self.goal_tolerance:.2f} m)\n'
            f'  LiDAR frente    : {front:.3f} m '
            f'(umbral={self.obstacle_threshold:.2f} m)\n'
            f'  h_min           : {self.h_min:.3f}\n'
            f'  d_followed      : {self.d_followed:.3f}\n'
            f'  d_reach         : {self.d_reach:.3f}\n'
            f'  BF side         : {self.bf_side}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TangentBugNode()
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
