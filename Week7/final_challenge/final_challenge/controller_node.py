"""Pure Pursuit — sigue el nav_msgs/Path del planner, publica cmd_vel."""
import math

import numpy as np
import rclpy
import transforms3d
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, ColorRGBA
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('lookahead_distance',     0.40)
        self.declare_parameter('lookahead_gain',         0.30)
        self.declare_parameter('max_linear',             0.15)
        self.declare_parameter('max_angular',            0.80)
        self.declare_parameter('kp_heading',             1.5)
        self.declare_parameter('rotate_in_place_thresh', 0.70)  # rad
        self.declare_parameter('goal_tolerance',         0.15)
        self.declare_parameter('control_rate_hz',        20.0)
        self.declare_parameter('map_frame',              'map')
        self.declare_parameter('robot_frame',            'base_footprint')

        self.lookahead_distance     = self.get_parameter('lookahead_distance').value
        self.lookahead_gain         = self.get_parameter('lookahead_gain').value
        self.max_linear             = self.get_parameter('max_linear').value
        self.max_angular            = self.get_parameter('max_angular').value
        self.kp_heading             = self.get_parameter('kp_heading').value
        self.rotate_in_place_thresh = self.get_parameter('rotate_in_place_thresh').value
        self.goal_tolerance         = self.get_parameter('goal_tolerance').value
        self.control_rate_hz        = self.get_parameter('control_rate_hz').value
        self.map_frame              = self.get_parameter('map_frame').value
        self.robot_frame            = self.get_parameter('robot_frame').value

        # ── Estado interno ────────────────────────────────────────────────────
        self.path   = None   # lista[(x, y)]
        self.goal   = None   # (x, y) en frame map
        self.last_v = 0.0    # última velocidad lineal publicada

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_path = QoSProfile(depth=1,  durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_goal = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.create_subscription(Path,        '/plan',      self.plan_cb, qos_path)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, qos_goal)

        # ── Publicadores ──────────────────────────────────────────────────────
        self.pub_cmd  = self.create_publisher(Twist,       '/cmd_vel',        10)
        self.pub_done = self.create_publisher(Bool,        '/goal_reached',   10)
        self.pub_viz  = self.create_publisher(MarkerArray, '/controller/viz', 10)

        self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def plan_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if self.path:
            self.get_logger().info(f'Plan recibido ({len(self.path)} puntos)')
        else:
            self.get_logger().warn('Plan vacío recibido')

    def goal_cb(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'Nuevo goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})'
        )

    # ── Consulta de pose ──────────────────────────────────────────────────────

    def _get_pose(self):
        """Devuelve (x, y, yaw) del robot en el frame del mapa, o None si no hay TF."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        _, _, yaw = transforms3d.euler.quat2euler(
            [q.w, q.x, q.y, q.z], axes='sxyz')
        return x, y, yaw

    # ── Loop de control ───────────────────────────────────────────────────────

    def control_loop(self):
        pose = self._get_pose()
        if pose is None:
            return
        x, y, yaw = pose

        # Comprobación de llegada al goal (independiente del plan)
        if self.goal is not None:
            d_goal = math.hypot(self.goal[0] - x, self.goal[1] - y)
            if d_goal < self.goal_tolerance:
                self._stop()
                self.pub_done.publish(Bool(data=True))
                self.path = None
                self.goal = None
                return

        if not self.path or len(self.path) < 2:
            self._stop()
            return

        # Lookahead adaptativo: a mayor velocidad, mira más lejos
        ld = self.lookahead_distance + self.lookahead_gain * abs(self.last_v)

        target = self._find_lookahead_point(x, y, ld)
        if target is None:
            target = self.path[-1]

        # Ángulo al target en el frame del robot
        dx = target[0] - x
        dy = target[1] - y
        d  = math.hypot(dx, dy)
        if d < 1e-3:
            self._stop()
            return

        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Si el error angular es grande, gira en sitio antes de avanzar
        if abs(alpha) > self.rotate_in_place_thresh:
            v = 0.0
            w = float(np.clip(self.kp_heading * alpha,
                              -self.max_angular, self.max_angular))
        else:
            # Curvatura Pure Pursuit: k = 2·sin(α) / ld
            k = 2.0 * math.sin(alpha) / max(ld, 0.05)
            v = self.max_linear
            w = float(np.clip(k * v, -self.max_angular, self.max_angular))
            # Reducción de velocidad en curvas pronunciadas
            v *= max(0.30, 1.0 - abs(w) / self.max_angular)

        self.last_v = v

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = w
        self.pub_cmd.publish(cmd)

        self._publish_viz(x, y, target, ld)

    # ── Punto de lookahead ────────────────────────────────────────────────────

    def _find_lookahead_point(self, x, y, ld):
        """Devuelve el primer punto del path a distancia >= ld desde la posición actual."""
        # Índice del punto más cercano al robot
        best_i = 0
        best_d = float('inf')
        for i, (px, py) in enumerate(self.path):
            d = (px - x) * (px - x) + (py - y) * (py - y)
            if d < best_d:
                best_d = d
                best_i = i
        # Primer punto delante que supera la distancia de lookahead
        for i in range(best_i, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= ld:
                return (px, py)
        return self.path[-1]

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        """Publica velocidad cero y reinicia el registro de velocidad."""
        self.last_v = 0.0
        self.pub_cmd.publish(Twist())

    def _publish_viz(self, x, y, target, ld):
        """Publica marcadores de visualización: punto de lookahead, línea y anillo."""
        arr   = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Esfera en el punto de lookahead
        sph = Marker()
        sph.header.frame_id = self.map_frame
        sph.header.stamp    = stamp
        sph.ns              = 'lookahead'
        sph.id              = 0
        sph.type            = Marker.SPHERE
        sph.action          = Marker.ADD
        sph.scale.x = sph.scale.y = sph.scale.z = 0.16
        sph.color           = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.95)
        sph.pose.position.x = target[0]
        sph.pose.position.y = target[1]
        sph.pose.position.z = 0.10
        sph.pose.orientation.w = 1.0
        arr.markers.append(sph)

        # Línea robot → lookahead
        ln = Marker()
        ln.header.frame_id = self.map_frame
        ln.header.stamp    = stamp
        ln.ns              = 'lookahead_line'
        ln.id              = 0
        ln.type            = Marker.LINE_STRIP
        ln.action          = Marker.ADD
        ln.scale.x         = 0.02
        ln.color           = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)
        ln.pose.orientation.w = 1.0
        ln.points = [Point(x=x, y=y, z=0.07),
                     Point(x=target[0], y=target[1], z=0.07)]
        arr.markers.append(ln)

        # Círculo con radio de lookahead
        ring = Marker()
        ring.header.frame_id = self.map_frame
        ring.header.stamp    = stamp
        ring.ns              = 'lookahead_ring'
        ring.id              = 0
        ring.type            = Marker.CYLINDER
        ring.action          = Marker.ADD
        ring.scale.x = ring.scale.y = 2.0 * ld
        ring.scale.z = 0.005
        ring.color   = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.10)
        ring.pose.position.x = x
        ring.pose.position.y = y
        ring.pose.position.z = 0.005
        ring.pose.orientation.w = 1.0
        arr.markers.append(ring)

        self.pub_viz.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
