"""
camera_node.py
==============
Nodo ROS 2 para detección de marcadores ArUco desde la cámara del Puzzlebot
en simulación Gazebo.

Suscripciones:
  /camera  (sensor_msgs/Image)      – imagen sin rectificar
  /camera_info (sensor_msgs/CameraInfo) – parámetros intrínsecos

Publicaciones:
  /aruco/detections   (final_challenge/ArucoDetection) – NO existe aún,
                       se publica como PoseArray por compatibilidad
  /aruco/image_debug  (sensor_msgs/Image)              – imagen anotada

Los datos de pose de cada marcador detectado se publican también en TF
como 'marker_<id>' referenciado al frame de la cámara.

Dependencias externas permitidas:
  opencv-contrib-python, numpy
"""

import numpy as np
import cv2
from cv2 import aruco

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header

from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import math


# ──────────────────────────────────────────────────────────────────────────────
# Utilidades de rotación
# ──────────────────────────────────────────────────────────────────────────────

def rotation_vector_to_quaternion(rvec: np.ndarray):
    """Convierte un vector de rotación de Rodrigues a cuaternión (x, y, z, w)."""
    angle = np.linalg.norm(rvec)
    if angle < 1e-10:
        return 0.0, 0.0, 0.0, 1.0
    axis = rvec.flatten() / angle
    s = math.sin(angle / 2.0)
    return (
        float(axis[0] * s),
        float(axis[1] * s),
        float(axis[2] * s),
        float(math.cos(angle / 2.0)),
    )


# ──────────────────────────────────────────────────────────────────────────────
# Nodo principal
# ──────────────────────────────────────────────────────────────────────────────

class CameraNode(Node):
    """
    Detecta marcadores ArUco en el stream de la cámara del Puzzlebot
    y publica su pose relativa al frame de la cámara.
    """

    def __init__(self):
        super().__init__('camera_node')

        # ── Parámetros declarados ─────────────────────────────────────────────
        self.declare_parameter('aruco_dict',    'DICT_4X4_250')
        self.declare_parameter('marker_size',   0.09)          # metros
        self.declare_parameter('camera_frame',  'camera_link_optical')
        self.declare_parameter('publish_debug', True)

        dict_name   = self.get_parameter('aruco_dict').value
        self.marker_size   = self.get_parameter('marker_size').value
        self.camera_frame  = self.get_parameter('camera_frame').value
        self.publish_debug = self.get_parameter('publish_debug').value

        # ── Diccionario ArUco ─────────────────────────────────────────────────
        dict_map = {
            'DICT_4X4_50':   aruco.DICT_4X4_50,
            'DICT_4X4_100':  aruco.DICT_4X4_100,
            'DICT_4X4_250':  aruco.DICT_4X4_250,
            'DICT_4X4_1000': aruco.DICT_4X4_1000,
            'DICT_5X5_50':   aruco.DICT_5X5_50,
            'DICT_5X5_250':  aruco.DICT_5X5_250,
        }

        aruco_dict_id = dict_map.get(dict_name, aruco.DICT_4X4_250)

        # NUEVA API
        self.dictionary = aruco.getPredefinedDictionary(aruco_dict_id)
        self.detector_params = aruco.DetectorParameters()

        # NUEVO detector
        self.detector = aruco.ArucoDetector(
            self.dictionary,
            self.detector_params
        )

        # ── Parámetros de cámara (se llenan con camera_info) ──────────────────
        self.camera_matrix = None
        self.dist_coeffs   = None
        self.camera_ready  = False

        # ── CV Bridge ─────────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── TF broadcaster ────────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── QoS ───────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Suscriptores ──────────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo,
            '/camera_info',
            self._camera_info_cb,
            sensor_qos,
        )
        self.create_subscription(
            Image,
            '/camera',
            self._image_cb,
            sensor_qos,
        )

        # ── Publicadores ──────────────────────────────────────────────────────
        # Pose de cada marcador detectado (frame de la cámara)
        self.pub_poses = self.create_publisher(PoseArray, '/aruco/detections', 10)

        # Imagen de debug con los marcadores dibujados
        if self.publish_debug:
            self.pub_debug = self.create_publisher(Image, '/aruco/image_debug', 1)

        self.get_logger().info(
            f'CameraNode listo  |  dict={dict_name}  '
            f'marker_size={self.marker_size} m  '
            f'frame={self.camera_frame}'
        )

    # ── Callback: CameraInfo ──────────────────────────────────────────────────

    def _camera_info_cb(self, msg: CameraInfo):
        """Guarda los parámetros intrínsecos la primera vez que llegan."""
        if self.camera_ready:
            return

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)

        # Si el modelo de distorsión es vacío (cámara ideal en Gazebo)
        if D.size == 0:
            D = np.zeros(5, dtype=np.float64)

        self.camera_matrix = K
        self.dist_coeffs   = D
        self.camera_ready  = True

        self.get_logger().info(
            f'CameraInfo recibida  fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  '
            f'cx={K[0,2]:.1f}  cy={K[1,2]:.1f}'
        )

    # ── Callback: Image ───────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        """Detecta marcadores en cada frame y publica resultados."""
        if not self.camera_ready:
            return

        # Convertir ROS Image → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'CvBridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── Detección ─────────────────────────────────────────────────────────
        corners, ids, rejected = self.detector.detectMarkers(gray)

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp    = msg.header.stamp
        pose_array.header.frame_id = self.camera_frame

        debug_frame = frame.copy() if self.publish_debug else None

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(debug_frame if debug_frame is not None else frame,
                                      corners, ids)

            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i]

                # ── Estimación de pose (PnP) ───────────────────────────────────
                # Coordenadas 3D del marcador
                obj_points = np.array([
                    [-self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2,  self.marker_size / 2, 0],
                    [ self.marker_size / 2, -self.marker_size / 2, 0],
                    [-self.marker_size / 2, -self.marker_size / 2, 0],
                ], dtype=np.float32)

                # Esquinas detectadas
                img_points = corner[0].astype(np.float32)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points,
                    img_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not success:
                    continue

                rvec = rvec.flatten()
                tvec = tvec.flatten()

                # ── Pose en PoseArray ─────────────────────────────────────────
                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])

                qx, qy, qz, qw = rotation_vector_to_quaternion(rvec)
                pose.orientation.x = qx
                pose.orientation.y = qy
                pose.orientation.z = qz
                pose.orientation.w = qw
                pose_array.poses.append(pose)

                # ── TF: marker_<id> en el frame de la cámara ─────────────────
                self._publish_marker_tf(
                    stamp=msg.header.stamp,
                    marker_id=int(marker_id),
                    tvec=tvec,
                    rvec=rvec,
                )

                # ── Distancia y anotación en debug ────────────────────────────
                dist = float(np.linalg.norm(tvec))

                if debug_frame is not None:
                    cv2.drawFrameAxes(
                        debug_frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.marker_size * 0.5,
                    )
                    # Centro del marcador
                    c = corner[0].mean(axis=0).astype(int)
                    cv2.putText(
                        debug_frame,
                        f'id={marker_id}  {dist:.2f}m',
                        (c[0], c[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

                self.get_logger().debug(
                    f'Marcador id={marker_id}  '
                    f'pos=({tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f})  '
                    f'dist={dist:.3f} m'
                )

        # ── Publicar poses (aunque sea vacío → señal de "ningún marcador") ────
        self.pub_poses.publish(pose_array)

        # ── Publicar imagen de debug ──────────────────────────────────────────
        if self.publish_debug and debug_frame is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding='bgr8')
                debug_msg.header = msg.header
                self.pub_debug.publish(debug_msg)
            except Exception as e:
                self.get_logger().warn(f'Debug image publish error: {e}')

    # ── TF helper ─────────────────────────────────────────────────────────────

    def _publish_marker_tf(self, stamp, marker_id: int,
                           tvec: np.ndarray, rvec: np.ndarray):
        """Publica la transformación cámara → marcador en TF."""
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id  = f'marker_{marker_id}'

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        qx, qy, qz, qw = rotation_vector_to_quaternion(rvec)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


# ──────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
