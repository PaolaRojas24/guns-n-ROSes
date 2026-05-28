import math
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


# ══════════════════════════════════════════════════════════════════════════════
# Utilidades
# ══════════════════════════════════════════════════════════════════════════════

def _normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al rango [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# ══════════════════════════════════════════════════════════════════════════════
# Nodo principal
# ══════════════════════════════════════════════════════════════════════════════

class LocalisationNode(Node):

    # IDs de marcadores esperados en todos los mundos
    _MARKER_IDS = (0, 1, 2, 3)

    def __init__(self):
        super().__init__('localisation_node')

        # ── Parámetros de cinemática ──────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base',   0.108)
        self.declare_parameter('kr',           0.014)
        self.declare_parameter('kl',           0.014)

        # Varianza de la observación ArUco: [dist² (m²), angle² (rad²)]
        self.declare_parameter('aruco_r_dist',  0.0025)   # ≈5cm std
        self.declare_parameter('aruco_r_angle', 0.0025)   # ≈3° std

        self.r  = self.get_parameter('wheel_radius').value
        self.L  = self.get_parameter('wheel_base').value
        self.kr = self.get_parameter('kr').value
        self.kl = self.get_parameter('kl').value

        self.r_dist  = self.get_parameter('aruco_r_dist').value
        self.r_angle = self.get_parameter('aruco_r_angle').value

        # ── Mapa de marcadores desde parámetros ROS 2 ─────────────────────────
        # Cada marcador expone tres parámetros:
        #   aruco_map.marker_<id>.x   (float)
        #   aruco_map.marker_<id>.y   (float)
        #   aruco_map.marker_<id>.yaw (float)
        self.aruco_map: dict = {}
        for mid in self._MARKER_IDS:
            prefix = f'aruco_map.marker_{mid}'
            self.declare_parameter(f'{prefix}.x',   0.0)
            self.declare_parameter(f'{prefix}.y',   0.0)
            self.declare_parameter(f'{prefix}.yaw', 0.0)

        self._reload_aruco_map()

        # ── Estado del robot ──────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.wr = 0.0   # velocidad angular rueda derecha [rad/s]
        self.wl = 0.0   # velocidad angular rueda izquierda [rad/s]

        # Covarianza 3×3  [x, y, θ]
        self.sigma = np.zeros((3, 3))

        self.prev_time = None

        # ── Publishers ────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, 'wr', self._wr_cb, 10)
        self.create_subscription(Float32, 'wl', self._wl_cb, 10)
        self.create_subscription(PoseArray, '/aruco/detections',
                                 self._aruco_cb, 10)

        # ── Timer de integración a 50 Hz ──────────────────────────────────────
        self.create_timer(0.02, self._update)

        self.get_logger().info(
            f'Localisation iniciado — r={self.r} m  L={self.L} m  '
            f'marcadores={list(self.aruco_map.keys())}'
        )

    # ── Carga del mapa desde parámetros ──────────────────────────────────────

    def _reload_aruco_map(self):
        """
        Lee aruco_map.marker_<id>.{x,y,yaw} de los parámetros ROS 2 y
        construye self.aruco_map = {id: {'x': float, 'y': float, 'yaw': float}}

        Solo registra los marcadores cuyos parámetros son distintos de cero
        (si x==0 y y==0 y yaw==0 asumimos que el mundo no declaró ese marcador
        y lo ignoramos para no contaminar el EKF).
        """
        self.aruco_map = {}
        for mid in self._MARKER_IDS:
            prefix = f'aruco_map.marker_{mid}'
            x   = self.get_parameter(f'{prefix}.x').value
            y   = self.get_parameter(f'{prefix}.y').value
            yaw = self.get_parameter(f'{prefix}.yaw').value

            # Ignorar marcadores no definidos (valores por defecto todos cero)
            if x == 0.0 and y == 0.0 and yaw == 0.0:
                continue

            self.aruco_map[mid] = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}

        if self.aruco_map:
            for mid, v in sorted(self.aruco_map.items()):
                self.get_logger().info(
                    f'  marker_{mid}: x={v["x"]:.2f}  y={v["y"]:.2f}'
                    f'  yaw={v["yaw"]:.4f} rad'
                )
        else:
            self.get_logger().warn(
                'aruco_map vacío — verifica que el config YAML del mundo '
                'se está pasando como parámetro en el launch file.'
            )

    # ── Callbacks de encoders ─────────────────────────────────────────────────

    def _wr_cb(self, msg: Float32):
        self.wr = msg.data

    def _wl_cb(self, msg: Float32):
        self.wl = msg.data

    # ── Loop de integración (predicción EKF) ──────────────────────────────────

    def _update(self):
        now = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        if dt <= 0.0:
            return

        # ── Cinemática diferencial ────────────────────────────────────────────
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L

        # ── Jacobiano H evaluado ANTES de integrar (yaw actual) ───────────────
        H = np.array([
            [1.0, 0.0, -dt * v * math.sin(self.yaw)],
            [0.0, 1.0,  dt * v * math.cos(self.yaw)],
            [0.0, 0.0,  1.0],
        ])

        # ── Integración de la pose ────────────────────────────────────────────
        self.x   += v * math.cos(self.yaw) * dt
        self.y   += v * math.sin(self.yaw) * dt
        self.yaw  = _normalize_angle(self.yaw + w * dt)

        # ── Jacobiano de velocidades ∇w (3×2) ────────────────────────────────
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        grad_w = np.array([
            [ (self.r / 2.0) * dt * c,   (self.r / 2.0) * dt * c],
            [ (self.r / 2.0) * dt * s,   (self.r / 2.0) * dt * s],
            [ self.r * dt / self.L,      -self.r * dt / self.L   ],
        ])

        # ── Ruido de proceso  Q = ∇w · Σ_Δ · ∇w^T ───────────────────────────
        sigma_delta = np.diag([
            self.kr * abs(self.wr),
            self.kl * abs(self.wl),
        ])
        Q = grad_w @ sigma_delta @ grad_w.T

        # ── Propagación de covarianza  Σ = H · Σ · H^T + Q ───────────────────
        self.sigma = H @ self.sigma @ H.T + Q

        # ── Publicar Odometry ─────────────────────────────────────────────────
        self._publish_odom(now, v, w)

    # ── Publicación ───────────────────────────────────────────────────────────

    def _publish_odom(self, stamp, v: float, w: float):
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Mapeo Σ 3×3 → covarianza 6×6 ROS (orden: x y z roll pitch yaw)
        cov36 = [0.0] * 36
        cov36[0]  = self.sigma[0, 0]   # x–x
        cov36[1]  = self.sigma[0, 1]   # x–y
        cov36[5]  = self.sigma[0, 2]   # x–θ
        cov36[6]  = self.sigma[1, 0]   # y–x
        cov36[7]  = self.sigma[1, 1]   # y–y
        cov36[11] = self.sigma[1, 2]   # y–θ
        cov36[30] = self.sigma[2, 0]   # θ–x
        cov36[31] = self.sigma[2, 1]   # θ–y
        cov36[35] = self.sigma[2, 2]   # θ–θ

        odom = Odometry()
        odom.header.stamp    = stamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance         = cov36

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

    # ── Helpers para el paso de corrección EKF ────────────────────────────────

    def expected_observation(self, marker_id: int):
        """
        Observación esperada h(x) para un marcador conocido.

        Modelo polar:  z = [ρ, α]
          ρ = distancia euclidiana robot → marcador  (m)
          α = ángulo al marcador relativo al heading del robot  (rad, [-π, π])

        Retorna (rho, alpha) o None si el marcador no está en el mapa.
        """
        if marker_id not in self.aruco_map:
            return None

        mx = self.aruco_map[marker_id]['x']
        my = self.aruco_map[marker_id]['y']

        dx    = mx - self.x
        dy    = my - self.y
        rho   = math.hypot(dx, dy)
        alpha = _normalize_angle(math.atan2(dy, dx) - self.yaw)

        return rho, alpha

    def observation_jacobian(self, marker_id: int):
        """
        Jacobiano H_obs (2×3) de h(x) respecto al estado [x, y, θ].

          ∂ρ/∂x = -(mx-x)/ρ       ∂ρ/∂y = -(my-y)/ρ     ∂ρ/∂θ =  0
          ∂α/∂x =  (my-y)/ρ²      ∂α/∂y = -(mx-x)/ρ²    ∂α/∂θ = -1

        Retorna np.ndarray (2×3) o None si el marcador no está en el mapa.
        """
        if marker_id not in self.aruco_map:
            return None

        mx = self.aruco_map[marker_id]['x']
        my = self.aruco_map[marker_id]['y']

        dx   = mx - self.x
        dy   = my - self.y
        rho2 = dx**2 + dy**2
        rho  = math.sqrt(rho2)

        if rho < 1e-6:
            return None

        return np.array([
            [-dx / rho,   -dy / rho,   0.0],
            [ dy / rho2,  -dx / rho2, -1.0],
        ])

    # ── Callback ArUco — paso de corrección EKF ──────────────────────────────

    def _aruco_cb(self, msg: PoseArray):
        """
        Corrección EKF por cada marcador detectado.

        msg.poses está indexado por marker_id (slots 0..3). Una pose con
        position.x == NaN significa "no detectado" — se ignora.

        La pose viene en el frame óptico de la cámara:
          x = derecha, y = abajo, z = adelante
        Se proyecta al plano del suelo (xz) para obtener (rho, alpha)
        relativos al robot. Se asume que la cámara está centrada en
        base_footprint (offset pequeño, absorbido en R).
        """
        R = np.diag([self.r_dist, self.r_angle])

        for mid in self._MARKER_IDS:
            if mid >= len(msg.poses):
                continue

            pose = msg.poses[mid]
            if math.isnan(pose.position.x):
                continue
            if mid not in self.aruco_map:
                continue

            cam_x = pose.position.x      # derecha (+) / izquierda (−)
            cam_z = pose.position.z      # adelante

            rho_meas   = math.hypot(cam_x, cam_z)
            alpha_meas = math.atan2(-cam_x, cam_z)   # +α = a la izquierda

            h_exp = self.expected_observation(mid)
            H_obs = self.observation_jacobian(mid)
            if h_exp is None or H_obs is None:
                continue
            rho_exp, alpha_exp = h_exp

            S = H_obs @ self.sigma @ H_obs.T + R
            try:
                K = self.sigma @ H_obs.T @ np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue

            inn = np.array([
                rho_meas - rho_exp,
                _normalize_angle(alpha_meas - alpha_exp),
            ])

            delta = K @ inn
            self.x   += delta[0]
            self.y   += delta[1]
            self.yaw  = _normalize_angle(self.yaw + delta[2])

            self.sigma = (np.eye(3) - K @ H_obs) @ self.sigma


# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()