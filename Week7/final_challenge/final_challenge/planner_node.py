"""Planner — A* sobre el OccupancyGrid de SLAM. Publica nav_msgs/Path."""
import heapq
import math
from collections import deque

import numpy as np
import rclpy
import transforms3d
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)
from scipy.ndimage import binary_dilation
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('robot_radius',       0.12)
        self.declare_parameter('replan_period_s',    1.5)
        self.declare_parameter('map_frame',          'map')
        self.declare_parameter('robot_frame',        'base_footprint')
        self.declare_parameter('unknown_is_obstacle', False)
        self.declare_parameter('path_simplify',      True)
        self.declare_parameter('allow_diagonal',     True)
        # Limpieza en vivo del grid con el LiDAR actual (anti paredes fantasma SLAM)
        self.declare_parameter('use_live_scan_clearing', True)
        self.declare_parameter('scan_topic',             '/scan')
        self.declare_parameter('clearing_max_range',     5.0)

        self.robot_radius         = self.get_parameter('robot_radius').value
        self.replan_period_s      = self.get_parameter('replan_period_s').value
        self.map_frame            = self.get_parameter('map_frame').value
        self.robot_frame          = self.get_parameter('robot_frame').value
        self.unknown_is_obstacle  = self.get_parameter('unknown_is_obstacle').value
        self.path_simplify        = self.get_parameter('path_simplify').value
        self.allow_diagonal       = self.get_parameter('allow_diagonal').value
        self.use_live_scan_clearing = self.get_parameter('use_live_scan_clearing').value
        self.scan_topic           = self.get_parameter('scan_topic').value
        self.clearing_max_range   = self.get_parameter('clearing_max_range').value

        self.map_msg          = None
        self.raw_obs          = None    # obstáculos crudos del SLAM (sin inflar)
        self.inflated_obs     = None    # SLAM inflado (para compatibilidad)
        self.work_obs         = None    # raw limpio con LiDAR + inflado (lo que usa A*)
        self.inflation_struct = None    # estructura circular precomputada para dilation
        self.last_scan        = None
        self.goal             = None    # (x, y) en frame map
        # Padding aplicado al grid de trabajo cuando start/goal caen fuera del mapa
        self._pad_row         = 0
        self._pad_col         = 0
        self.declare_parameter('out_of_map_margin_cells', 10)
        self.out_of_map_margin = self.get_parameter('out_of_map_margin_cells').value

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos_map  = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_goal = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_path = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(OccupancyGrid, '/map',       self.map_cb,  qos_map)
        self.create_subscription(PoseStamped,   '/goal_pose', self.goal_cb, qos_goal)
        if self.use_live_scan_clearing:
            self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.pub_plan = self.create_publisher(Path, '/plan',          qos_path)
        self.pub_viz  = self.create_publisher(MarkerArray, '/planner/viz', 10)

        self.create_timer(self.replan_period_s, self.replan)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        self._update_inflated()

    def goal_cb(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f'Nuevo goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})'
        )
        self.replan()

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    # ── Limpieza en vivo con el LiDAR ────────────────────────────────────────

    def _apply_live_scan_clearing(self, base_obs):
        """
        Devuelve una COPIA de base_obs en la que se marcan como FALSE (libre)
        las celdas que el LiDAR actual ve como libres (Bresenham desde el
        robot hasta cada hit, sin tocar la celda del hit).
        """
        if (not self.use_live_scan_clearing or self.last_scan is None
                or self.map_msg is None):
            return base_obs

        # Pose del láser en el frame del mapa
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, 'laser_frame', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return base_obs

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        q  = t.transform.rotation
        _, _, ryaw = transforms3d.euler.quat2euler(
            [q.w, q.x, q.y, q.z], axes='sxyz')

        info = self.map_msg.info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        rc = int((ry - oy) / res)
        cc = int((rx - ox) / res)

        cleared = base_obs.copy()
        h, w    = cleared.shape

        scan        = self.last_scan
        ranges      = np.asarray(scan.ranges, dtype=float)
        n           = len(ranges)
        if n == 0 or scan.angle_increment <= 0.0:
            return cleared
        angles = scan.angle_min + np.arange(n) * scan.angle_increment

        max_r = scan.range_max if scan.range_max > 0 else self.clearing_max_range
        for r, theta in zip(ranges, angles):
            if not np.isfinite(r) or r <= 0.01:
                # Rayo inválido: limpia hasta el max_range razonable
                clear_r = self.clearing_max_range
                hit_at_end = False
            elif r >= max_r - 0.05:
                # Rayo no encontró obstáculo: limpia hasta clearing_max_range
                clear_r = min(r, self.clearing_max_range)
                hit_at_end = False
            else:
                # Hit real: limpia hasta antes del hit, deja la celda del hit intacta
                clear_r = min(r, self.clearing_max_range)
                hit_at_end = True
            ex = rx + clear_r * math.cos(ryaw + theta)
            ey = ry + clear_r * math.sin(ryaw + theta)
            ec = int((ex - ox) / res)
            er = int((ey - oy) / res)
            self._bresenham_clear(cleared, rc, cc, er, ec, h, w,
                                  include_endpoint=not hit_at_end)
        return cleared

    @staticmethod
    def _bresenham_clear(grid, r0, c0, r1, c1, h, w, include_endpoint=False):
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc
        while True:
            if r0 == r1 and c0 == c1:
                if include_endpoint and 0 <= r0 < h and 0 <= c0 < w:
                    grid[r0, c0] = False
                break    # no limpiar la celda del hit (a menos que include_endpoint)
            if 0 <= r0 < h and 0 <= c0 < w:
                grid[r0, c0] = False
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r0  += sr
            if e2 < dr:
                err += dr
                c0  += sc

    def _stamp_live_obstacles(self, grid):
        """
        Marca como obstáculo (True) las celdas donde el LiDAR ve un hit real.
        Trabaja sobre el grid YA EXPANDIDO usando los offsets de padding, de
        modo que los obstáculos vistos fuera del mapa de SLAM también cuenten.
        Devuelve una copia para no mutar self.raw_obs.
        """
        if (not self.use_live_scan_clearing or self.last_scan is None
                or self.map_msg is None):
            return grid
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, 'laser_frame', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return grid

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        q  = t.transform.rotation
        _, _, ryaw = transforms3d.euler.quat2euler(
            [q.w, q.x, q.y, q.z], axes='sxyz')

        info = self.map_msg.info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        scan   = self.last_scan
        ranges = np.asarray(scan.ranges, dtype=float)
        n      = len(ranges)
        if n == 0 or scan.angle_increment <= 0.0:
            return grid
        angles = scan.angle_min + np.arange(n) * scan.angle_increment
        max_r  = scan.range_max if scan.range_max > 0 else self.clearing_max_range

        grid = grid.copy()
        h, w = grid.shape
        for r, theta in zip(ranges, angles):
            if not np.isfinite(r) or r <= 0.01:
                continue                      # rayo inválido
            if r >= max_r - 0.05 or r > self.clearing_max_range:
                continue                      # no encontró obstáculo
            ex = rx + r * math.cos(ryaw + theta)
            ey = ry + r * math.sin(ryaw + theta)
            ec = int((ex - ox) / res) + self._pad_col
            er = int((ey - oy) / res) + self._pad_row
            if 0 <= er < h and 0 <= ec < w:
                grid[er, ec] = True
        return grid

    # ── Procesamiento del mapa ────────────────────────────────────────────────

    def _update_inflated(self):
        """Guarda obstáculos crudos del SLAM y precomputa la estructura de inflación."""
        info = self.map_msg.info
        h, w = info.height, info.width
        data = np.array(self.map_msg.data, dtype=np.int8).reshape(h, w)
        raw  = data >= 50
        if self.unknown_is_obstacle:
            raw |= (data < 0)
        self.raw_obs = raw

        # Estructura circular para inflar al radio del robot
        radius_cells = max(1, int(math.ceil(self.robot_radius / info.resolution)))
        y, x = np.ogrid[-radius_cells:radius_cells + 1,
                        -radius_cells:radius_cells + 1]
        self.inflation_struct = (x * x + y * y) <= (radius_cells * radius_cells)

        # Versión inflada del SLAM (sin limpieza LiDAR) — por compatibilidad
        self.inflated_obs = binary_dilation(raw, structure=self.inflation_struct)

    def _world_to_grid(self, wx, wy):
        info = self.map_msg.info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        return row, col

    def _grid_to_world(self, row, col):
        info = self.map_msg.info
        wx = info.origin.position.x + (col - self._pad_col + 0.5) * info.resolution
        wy = info.origin.position.y + (row - self._pad_row + 0.5) * info.resolution
        return wx, wy

    def _get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    # ── A* ────────────────────────────────────────────────────────────────────

    def replan(self):
        if self.map_msg is None or self.goal is None or self.raw_obs is None:
            return
        start_xy = self._get_robot_xy()
        if start_xy is None:
            return

        # 1) limpieza LiDAR sobre obstáculos CRUDOS (no inflados)
        cleaned_raw = self._apply_live_scan_clearing(self.raw_obs)

        # coords (pueden caer fuera del grid: negativas o >= h/w)
        start_rc = self._world_to_grid(*start_xy)
        goal_rc  = self._world_to_grid(*self.goal)

        # 2) agrandar el grid CRUDO para abarcar start y goal aunque estén
        #    fuera del mapa de SLAM (espacio nuevo = libre/navegable)
        expanded_raw, start_rc, goal_rc = self._expand_to_include(
            cleaned_raw, start_rc, goal_rc)

        # 3) estampar los hits del LiDAR como obstáculos: así las paredes que
        #    el láser ve FUERA del mapa de SLAM (en la zona recién añadida)
        #    también se respetan y la ruta no las atraviesa.
        expanded_raw = self._stamp_live_obstacles(expanded_raw)

        # 4) inflar DESPUÉS de expandir/estampar: las paredes (de SLAM o del
        #    LiDAR) extienden su zona de seguridad al área nueva y la ruta no se
        #    "cuela" por el extremo cortado de una pared.
        self.work_obs = binary_dilation(expanded_raw, structure=self.inflation_struct)

        start_rc = self._nearest_free(start_rc)
        goal_rc  = self._nearest_free(goal_rc)

        path_cells = self._astar(start_rc, goal_rc)
        if path_cells is None:
            self.get_logger().warn(
                f'No se encontró ruta start={start_xy} goal={self.goal}'
            )
            self._publish_empty_plan()
            return

        if self.path_simplify:
            path_cells = self._simplify(path_cells)

        path_world = [self._grid_to_world(r, c) for (r, c) in path_cells]
        self._publish_plan(path_world)
        self._publish_viz(path_world)

    def _nearest_free(self, rc):
        obs = self.work_obs if self.work_obs is not None else self.inflated_obs
        if not obs[rc[0], rc[1]]:
            return rc
        h, w = obs.shape
        q = deque([rc])
        seen = {rc}
        while q:
            r, c = q.popleft()
            if not obs[r, c]:
                return (r, c)
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and (nr, nc) not in seen:
                    seen.add((nr, nc))
                    q.append((nr, nc))
        return rc

    def _expand_to_include(self, grid, *cells):
        """
        Devuelve (grid_expandido, *cells_ajustadas) de forma que todas las
        celdas dadas queden dentro de los límites, con un margen alrededor.
        Opera sobre obstáculos CRUDOS (sin inflar): la inflación se aplica
        después, por el llamador, sobre el grid ya expandido.
        Las celdas nuevas se rellenan según unknown_is_obstacle (por defecto
        libres → navegables), permitiendo planificar a puntos fuera del mapa.
        Actualiza self._pad_row/_pad_col para las conversiones grid↔world.
        """
        h, w = grid.shape
        m = self.out_of_map_margin
        rows = [c[0] for c in cells]
        cols = [c[1] for c in cells]

        min_r = min(0, min(rows) - m)
        max_r = max(h - 1, max(rows) + m)
        min_c = min(0, min(cols) - m)
        max_c = max(w - 1, max(cols) + m)

        pad_r = -min_r          # filas añadidas antes del origen original
        pad_c = -min_c
        self._pad_row = pad_r
        self._pad_col = pad_c

        new_h = max_r - min_r + 1
        new_w = max_c - min_c + 1
        if new_h == h and new_w == w:
            return (grid, *cells)

        self.get_logger().info(
            f'Goal/start fuera del mapa: ampliando grid {w}x{h} → {new_w}x{new_h}'
        )
        fill = bool(self.unknown_is_obstacle)   # False = libre, True = obstáculo
        new_grid = np.full((new_h, new_w), fill, dtype=bool)
        new_grid[pad_r:pad_r + h, pad_c:pad_c + w] = grid

        adj = tuple((r + pad_r, c + pad_c) for (r, c) in cells)
        return (new_grid, *adj)

    def _astar(self, start, goal):
        obs = self.work_obs if self.work_obs is not None else self.inflated_obs
        if start == goal:
            return [start]
        h, w = obs.shape
        if self.allow_diagonal:
            moves = [(-1, -1, 1.4142), (-1, 0, 1.0), (-1, 1, 1.4142),
                     ( 0, -1, 1.0),                   ( 0, 1, 1.0),
                     ( 1, -1, 1.4142), ( 1, 0, 1.0), ( 1, 1, 1.4142)]
        else:
            moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

        def heur(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_heap = [(heur(start, goal), 0.0, start, None)]
        came      = {}
        g_score   = {start: 0.0}

        while open_heap:
            _, g, node, parent = heapq.heappop(open_heap)
            if node in came:
                continue
            came[node] = parent
            if node == goal:
                path = [node]
                while came[path[-1]] is not None:
                    path.append(came[path[-1]])
                path.reverse()
                return path
            r, c = node
            for dr, dc, cost in moves:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w):
                    continue
                if obs[nr, nc]:
                    continue
                new_g = g + cost
                if new_g < g_score.get((nr, nc), float('inf')):
                    g_score[(nr, nc)] = new_g
                    f_new = new_g + heur((nr, nc), goal)
                    heapq.heappush(open_heap, (f_new, new_g, (nr, nc), node))
        return None

    # ── Simplificación line-of-sight (Bresenham) ─────────────────────────────

    def _simplify(self, cells):
        if len(cells) < 3:
            return cells
        out = [cells[0]]
        anchor = 0
        for i in range(2, len(cells)):
            if not self._line_clear(cells[anchor], cells[i]):
                out.append(cells[i - 1])
                anchor = i - 1
        out.append(cells[-1])
        return out

    def _line_clear(self, a, b):
        obs = self.work_obs if self.work_obs is not None else self.inflated_obs
        r0, c0 = a
        r1, c1 = b
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        sr = 1 if r1 > r0 else -1
        sc = 1 if c1 > c0 else -1
        err = dr - dc
        h, w = obs.shape
        while True:
            if not (0 <= r0 < h and 0 <= c0 < w):
                return False
            if obs[r0, c0]:
                return False
            if r0 == r1 and c0 == c1:
                return True
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r0  += sr
            if e2 < dr:
                err += dr
                c0  += sc

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_plan(self, path_world):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp    = self.get_clock().now().to_msg()
        for (x, y) in path_world:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.pub_plan.publish(msg)

    def _publish_empty_plan(self):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.pub_plan.publish(msg)

    def _publish_viz(self, path_world):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        ls = Marker()
        ls.header.frame_id = self.map_frame
        ls.header.stamp    = stamp
        ls.ns              = 'path'
        ls.id              = 0
        ls.type            = Marker.LINE_STRIP
        ls.action          = Marker.ADD
        ls.scale.x         = 0.04
        ls.color           = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9)
        ls.pose.orientation.w = 1.0
        for (x, y) in path_world:
            ls.points.append(Point(x=x, y=y, z=0.06))
        arr.markers.append(ls)

        wps = Marker()
        wps.header.frame_id = self.map_frame
        wps.header.stamp    = stamp
        wps.ns              = 'path_pts'
        wps.id              = 0
        wps.type            = Marker.SPHERE_LIST
        wps.action          = Marker.ADD
        wps.scale.x = wps.scale.y = wps.scale.z = 0.08
        wps.color   = ColorRGBA(r=0.0, g=0.7, b=0.0, a=0.7)
        wps.pose.orientation.w = 1.0
        for (x, y) in path_world:
            wps.points.append(Point(x=x, y=y, z=0.06))
        arr.markers.append(wps)
        self.pub_viz.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
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
