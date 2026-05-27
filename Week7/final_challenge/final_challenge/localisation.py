import math
import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


# ══════════════════════════════════════════════════════════════════════════════
# Utilidades
# ══════════════════════════════════════════════════════════════════════════════

def _normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al rango [-π, π]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _load_aruco_map(path: str) -> dict:
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f'[localisation] aruco_map.yaml no encontrado: {path}'
        )

    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    raw = data.get('aruco_map', {})
    marker_map: dict = {}

    for name, values in raw.items():
        # Acepta "marker_0", "marker_12", etc.
        try:
            marker_id = int(name.split('_')[-1])
        except ValueError:
            continue

        marker_map[marker_id] = {
            'x':   float(values['x']),
            'y':   float(values['y']),
            'yaw': float(values.get('yaw', 0.0)),
        }

    return marker_map


# ══════════════════════════════════════════════════════════════════════════════
# Nodo principal
# ══════════════════════════════════════════════════════════════════════════════

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('localisation_node')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius',   0.05)
        self.declare_parameter('wheel_base',     0.108)
        self.declare_parameter('kr',             0.014)
        self.declare_parameter('kl',             0.014)
        self.declare_parameter('aruco_map_file', '')   # ruta al aruco_map.yaml

        self.r  = self.get_parameter('wheel_radius').value
        self.L  = self.get_parameter('wheel_base').value
        self.kr = self.get_parameter('kr').value
        self.kl = self.get_parameter('kl').value

        # ── Mapa de marcadores ────────────────────────────────────────────────
        map_file = self.get_parameter('aruco_map_file').value

        # Ruta por defecto: mismo directorio que este script → ../../config/
        if not map_file:
            pkg_dir   = os.path.dirname(os.path.abspath(__file__))
            map_file  = os.path.join(
                pkg_dir, '..', '..', '..', '..', 'share',
                'final_challenge', 'config', 'aruco_map.yaml'
            )
            map_file  = os.path.normpath(map_file)

        try:
            self.aruco_map: dict = _load_aruco_map(map_file)
            self.get_logger().info(
                f'Mapa ArUco cargado ({len(self.aruco_map)} marcadores): '
                + ', '.join(
                    f'id={k} @ ({v["x"]:.2f}, {v["y"]:.2f})'
                    for k, v in sorted(self.aruco_map.items())
                )
            )
        except FileNotFoundError as e:
            self.get_logger().warn(str(e))
            self.aruco_map = {}

        # ── Estado del robot ──────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.wr = 0.0   # velocidad angular rueda derecha [rad/s]
        self.wl = 0.0   # velocidad angular rueda izquierda [rad/s]

        # Covarianza 3×3   [xx, xy, xθ]
        #                  [yx, yy, yθ]
        #                  [θx, θy, θθ]
        self.sigma = np.zeros((3, 3))

        self.prev_time = None

        # ── Publishers ────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32, 'wr', self.wr_cb, 10)
        self.create_subscription(Float32, 'wl', self.wl_cb, 10)

        # TODO (EKF corrección): agregar suscripción a /aruco/detections
        # self.create_subscription(PoseArray, '/aruco/detections',
        #                          self.aruco_cb, 10)

        # ── Timer (loop de integración a 50 Hz) ───────────────────────────────
        self.create_timer(0.02, self.update)

        self.get_logger().info(
            f'Dead reckoning iniciado — r={self.r} m  L={self.L} m'
        )

    # ── Callbacks de encoders ─────────────────────────────────────────────────

    def wr_cb(self, msg: Float32):
        self.wr = msg.data

    def wl_cb(self, msg: Float32):
        self.wl = msg.data

    # ── Loop de integración (predicción EKF) ──────────────────────────────────

    def update(self):
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
        w = self.r * (self.wl - self.wr) / self.L

        # ── Jacobiano H ANTES de actualizar el estado  ← bug corregido ────────
        # (se evalúa con el yaw actual, no con el yaw ya integrado)
        H = np.array([
            [1.0, 0.0, -dt * v * math.sin(self.yaw)],
            [0.0, 1.0,  dt * v * math.cos(self.yaw)],
            [0.0, 0.0,  1.0],
        ])

        # ── Integración de la pose ────────────────────────────────────────────
        self.x   += v * math.cos(self.yaw + math.pi / 2.0) * dt
        self.y   += v * math.sin(self.yaw + math.pi / 2.0) * dt
        self.yaw  = _normalize_angle(self.yaw + w * dt)

        # ── Jacobiano de velocidades (3×2) ────────────────────────────────────
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        grad_w = np.array([
            [ (self.r / 2.0) * dt * c,  (self.r / 2.0) * dt * c],
            [ (self.r / 2.0) * dt * s,  (self.r / 2.0) * dt * s],
            [ self.r * dt / self.L,     -self.r * dt / self.L  ],
        ])

        # ── Ruido de proceso  Q = ∇ω · Σ_Δ · ∇ω^T ───────────────────────────
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
        # Cuaternión (corrección de orientación del URDF)
        yaw_q = self.yaw + math.pi / 2.0
        qz    = math.sin(yaw_q / 2.0)
        qw    = math.cos(yaw_q / 2.0)

        # Mapeo 3×3 → 6×6 (orden ROS: x y z roll pitch yaw)
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

    # ── Helpers públicos para el paso de corrección EKF ──────────────────────
    # Estos métodos ya están listos; solo necesitan ser llamados desde aruco_cb.

    def expected_observation(self, marker_id: int):
        """
        Calcula la observación esperada h(x) para un marcador conocido.

        Modelo polar:  z = [ρ, α]
          ρ = distancia euclidiana robot → marcador
          α = ángulo relativo al heading del robot, en [-π, π]

        Retorna (rho, alpha) o None si el marcador no está en el mapa.
        """
        if marker_id not in self.aruco_map:
            return None

        mx = self.aruco_map[marker_id]['x']
        my = self.aruco_map[marker_id]['y']

        dx  = mx - self.x
        dy  = my - self.y
        rho = math.hypot(dx, dy)

        # Ángulo absoluto hacia el marcador, relativo al heading del robot
        alpha = _normalize_angle(math.atan2(dy, dx) - self.yaw)

        return rho, alpha

    def observation_jacobian(self, marker_id: int):
        """
        Jacobiano H_obs (2×3) de h(x) respecto al estado [x, y, θ].

        ∂ρ/∂x  = -(mx-x)/ρ          ∂ρ/∂y  = -(my-y)/ρ    ∂ρ/∂θ = 0
        ∂α/∂x  =  (my-y)/ρ²         ∂α/∂y  = -(mx-x)/ρ²   ∂α/∂θ = -1

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

        if rho < 1e-6:          # marcador coincide con el robot → inestable
            return None

        H_obs = np.array([
            [-dx / rho,  -dy / rho,   0.0],
            [ dy / rho2, -dx / rho2, -1.0],
        ])

        return H_obs

    # TODO: implementar aruco_cb con el paso de corrección EKF
    #
    # def aruco_cb(self, msg: PoseArray):
    #     """
    #     Corrección EKF usando detecciones de ArUco.
    #
    #     Por cada marcador detectado en msg.poses (indexado por su id):
    #       1. Obtener la observación real z = [rho_medido, alpha_medido]
    #          a partir de la pose en el frame de la cámara.
    #       2. h_x, H_obs = expected_observation(id), observation_jacobian(id)
    #       3. Ruido de observación R (2×2), parámetro del nodo.
    #       4. S   = H_obs @ self.sigma @ H_obs.T + R
    #       5. K   = self.sigma @ H_obs.T @ np.linalg.inv(S)
    #       6. inn = z - h_x      (innovación, normalizar alpha)
    #       7. dx  = K @ inn
    #       8. self.x   += dx[0]
    #          self.y   += dx[1]
    #          self.yaw  = _normalize_angle(self.yaw + dx[2])
    #       9. self.sigma = (np.eye(3) - K @ H_obs) @ self.sigma
    #     """
    #     pass


# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
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