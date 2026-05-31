"""SLAM Resetter — solicita a slam_toolbox que limpie su estado (clear) y empiece de cero"""
import math

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node


class SlamResetterNode(Node):

    def __init__(self):
        super().__init__('slam_resetter')

        # ── Parámetros ────────────────────────────────────────────────────────
        self.declare_parameter('reset_on_aruco',          True)
        self.declare_parameter('periodic_reset_period_s', 0.0)   # 0 = desactivado
        self.declare_parameter('min_reset_interval_s',    5.0)   # debounce
        self.declare_parameter('initial_grace_s',         8.0)   # warm-up SLAM
        self.declare_parameter('reset_service',           '/slam_toolbox/clear')

        self.reset_on_aruco  = self.get_parameter('reset_on_aruco').value
        self.periodic_period = self.get_parameter('periodic_reset_period_s').value
        self.min_interval    = self.get_parameter('min_reset_interval_s').value
        self.initial_grace   = self.get_parameter('initial_grace_s').value
        reset_srv_name       = self.get_parameter('reset_service').value

        # ── Cliente del servicio ──────────────────────────────────────────────
        from slam_toolbox.srv import Clear as SlamClear
        self.reset_client    = self.create_client(SlamClear, reset_srv_name)
        self.reset_req_class = SlamClear.Request
        self.get_logger().info(f'Usando servicio: {reset_srv_name} (slam_toolbox/srv/Clear)')

        # ── Estado interno ────────────────────────────────────────────────────
        self.start_t      = self._now_s()
        self.last_reset_t = 0.0
        self.warn_count   = 0   # contador para throttle de warnings

        # ── Suscriptores y timers según configuración ─────────────────────────
        if self.reset_on_aruco:
            self.create_subscription(PoseArray, '/aruco/detections', self.aruco_cb, 10)
            self.get_logger().info('Reset por ArUco: ACTIVO')

        if self.periodic_period > 0:
            self.create_timer(self.periodic_period, self.periodic_reset)
            self.get_logger().info(
                f'Reset periódico cada {self.periodic_period:.1f} s'
            )

        self.get_logger().info(
            f'Debounce: {self.min_interval:.1f} s  |  '
            f'Grace inicial: {self.initial_grace:.1f} s'
        )

        # Diagnóstico de disponibilidad del servicio (no bloqueante)
        self._service_ready_logged = False
        self.create_timer(2.0, self._check_service_status)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _any_aruco_visible(self, msg: PoseArray) -> bool:
        """Devuelve True si al menos una pose del mensaje no es NaN."""
        for pose in msg.poses:
            if not math.isnan(pose.position.x):
                return True
        return False

    def _check_service_status(self):
        """Avisa una sola vez cuando el servicio de clear está disponible."""
        if self._service_ready_logged:
            return
        if self.reset_client.service_is_ready():
            self.get_logger().info('✓ Servicio /slam_toolbox/clear DISPONIBLE')
            self._service_ready_logged = True

    def _try_reset(self, reason: str):
        """
        Lanza la llamada asíncrona al servicio de clear si se superan los
        requisitos de grace inicial y debounce. No bloquea el hilo del nodo.
        """
        now = self._now_s()
        if now - self.start_t < self.initial_grace:
            return
        if now - self.last_reset_t < self.min_interval:
            return

        req = self.reset_req_class()
        try:
            future = self.reset_client.call_async(req)
        except Exception as e:
            self._throttled_warn(f'call_async lanzó excepción: {e}')
            return

        future.add_done_callback(lambda f: self._on_reset_done(f, reason))
        self.last_reset_t = now
        self.get_logger().info(f'⟳ SLAM clear solicitado  ({reason})')

    def _throttled_warn(self, msg: str):
        """Emite el warning solo 1 de cada 20 llamadas para evitar spam en el log."""
        self.warn_count += 1
        if self.warn_count % 20 == 1:
            self.get_logger().warn(msg)

    def _on_reset_done(self, future, reason: str = ''):
        """Callback del future: registra errores sin interrumpir el nodo."""
        try:
            result = future.result()
            if result is None:
                self._throttled_warn(
                    f'SLAM clear future devolvió None (servicio sin respuesta) — {reason}'
                )
                return
            success = getattr(result, 'success', True)
            if not success:
                self._throttled_warn(f'SLAM clear devolvió success=false — {reason}')
        except Exception as e:
            self._throttled_warn(f'SLAM clear falló ({reason}): {e}')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def aruco_cb(self, msg: PoseArray):
        if self._any_aruco_visible(msg):
            self._try_reset('aruco detectado')

    def periodic_reset(self):
        self._try_reset('periódico')


def main(args=None):
    rclpy.init(args=args)
    node = SlamResetterNode()
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
