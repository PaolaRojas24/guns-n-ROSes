import math
import numpy as np
import rclpy
import transforms3d

from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


GO_TO_GOAL      = 'GO_TO_GOAL'
FOLLOW_BOUNDARY = 'FOLLOW_BOUNDARY'
RETURN_TO_LEAVE = 'RETURN_TO_LEAVE'
GOAL_REACHED    = 'GOAL_REACHED'

_N_GOALS = 4


class Bug1Node(Node):

    def __init__(self):
        super().__init__('bug1_node')

        # --------------------------------------------------
        # Parámetros
        # --------------------------------------------------
        self.declare_parameter('obstacle_threshold', 0.40)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('forward_fov_deg', 40.0)
        self.declare_parameter('side_fov_deg', 30.0)

        self.declare_parameter('wall_dist_target', 0.35)
        self.declare_parameter('wall_linear_speed', 0.12)
        self.declare_parameter('wall_angular_speed', 0.60)
        self.declare_parameter('kp_wall', 1.20)

        self.declare_parameter('hit_radius', 0.25)
        self.declare_parameter('leave_radius', 0.25)
        self.declare_parameter('min_boundary_length', 1.0)

        for i in range(1, _N_GOALS + 1):
            self.declare_parameter(
                f'trajectory_goals.goal_{i}.x', 0.0)
            self.declare_parameter(
                f'trajectory_goals.goal_{i}.y', 0.0)

        self._load_params()

        # --------------------------------------------------
        # Waypoints
        # --------------------------------------------------
        self.waypoints = self._load_waypoints()
        self.waypoint_index = 0

        if len(self.waypoints) == 0:
            self.get_logger().error(
                'No se encontraron waypoints'
            )

        # --------------------------------------------------
        # Estado robot
        # --------------------------------------------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scan_front = float('inf')
        self.scan_right = float('inf')
        self.scan_ready = False

        self.state = GO_TO_GOAL

        self._pid_goal_sent = False

        # --------------------------------------------------
        # Variables Bug1
        # --------------------------------------------------
        self.hit_point = None
        self.leave_point = None

        self.best_goal_dist = float('inf')

        self.boundary_start_x = 0.0
        self.boundary_start_y = 0.0

        self.travelled_on_boundary = 0.0
        self.boundary_completed = False

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------
        self.pub_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pub_setpoint = self.create_publisher(
            Point,
            'setpoint',
            10
        )

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.create_subscription(
            Odometry,
            'odom',
            self.odom_cb,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            10
        )

        # --------------------------------------------------
        # Timers
        # --------------------------------------------------
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.debug_log)

        self.get_logger().info('Bug1 Node iniciado')

    # ======================================================
    # PARAMS
    # ======================================================

    def _load_params(self):

        self.obstacle_threshold = self.get_parameter(
            'obstacle_threshold').value

        self.goal_tolerance = self.get_parameter(
            'goal_tolerance').value

        self.forward_fov_deg = self.get_parameter(
            'forward_fov_deg').value

        self.side_fov_deg = self.get_parameter(
            'side_fov_deg').value

        self.wall_dist_target = self.get_parameter(
            'wall_dist_target').value

        self.wall_linear_speed = self.get_parameter(
            'wall_linear_speed').value

        self.wall_angular_speed = self.get_parameter(
            'wall_angular_speed').value

        self.kp_wall = self.get_parameter(
            'kp_wall').value

        self.hit_radius = self.get_parameter(
            'hit_radius').value

        self.leave_radius = self.get_parameter(
            'leave_radius').value

        self.min_boundary_length = self.get_parameter(
            'min_boundary_length').value

    # ======================================================

    def _load_waypoints(self):

        waypoints = []

        for i in range(1, _N_GOALS + 1):

            x = self.get_parameter(
                f'trajectory_goals.goal_{i}.x').value

            y = self.get_parameter(
                f'trajectory_goals.goal_{i}.y').value

            if x == 0.0 and y == 0.0:
                continue

            waypoints.append((float(x), float(y)))

        return waypoints

    @property
    def goal_x(self):
        return self.waypoints[self.waypoint_index][0]

    @property
    def goal_y(self):
        return self.waypoints[self.waypoint_index][1]

    # ======================================================

    def distance(self, p1, p2):
        return math.hypot(
            p1[0] - p2[0],
            p1[1] - p2[1]
        )

    def _dist_to_goal(self):
        return math.hypot(
            self.goal_x - self.x,
            self.goal_y - self.y
        )

    @staticmethod
    def _sector_min(
            ranges,
            center_idx,
            half_width,
            n):

        idxs = [
            (center_idx + i) % n
            for i in range(
                -half_width,
                half_width + 1)
        ]

        vals = ranges[idxs]
        vals = vals[(vals > 0.01) & np.isfinite(vals)]

        return float(np.min(vals)) \
            if len(vals) else float('inf')

    # ======================================================

    def _cmd(self, linear, angular):

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.pub_cmd.publish(msg)

    def _send_setpoint(self, x, y):

        self.get_logger().info(
            f'Publicando setpoint ({x:.2f}, {y:.2f})'
        )

        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = 0.0

        self.pub_setpoint.publish(msg)

    def _cancel_pid(self):

        self._send_setpoint(self.x, self.y)
        self._cmd(0.0, 0.0)

        self._pid_goal_sent = False

    # ======================================================
    # CALLBACKS
    # ======================================================

    def odom_cb(self, msg):

        old_x = self.x
        old_y = self.y

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        _, _, self.yaw = (
            transforms3d.euler.quat2euler(
                [q.w, q.x, q.y, q.z],
                axes='sxyz')
        )

        if self.state in [
            FOLLOW_BOUNDARY,
            RETURN_TO_LEAVE
        ]:

            self.travelled_on_boundary += math.hypot(
                self.x - old_x,
                self.y - old_y
            )

    def scan_cb(self, msg):

        ranges = np.array(msg.ranges)

        n = len(ranges)

        inc = msg.angle_increment

        half_fwd = max(
            1,
            int(
                math.radians(
                    self.forward_fov_deg / 2)
                / inc
            )
        )

        half_side = max(
            1,
            int(
                math.radians(
                    self.side_fov_deg / 2)
                / inc
            )
        )

        idx_right = int(
            round(math.radians(-90) / inc)
        ) % n

        self.scan_front = self._sector_min(
            ranges,
            0,
            half_fwd,
            n
        )

        self.scan_right = self._sector_min(
            ranges,
            idx_right,
            half_side,
            n
        )

        self.scan_ready = True

    # ======================================================
    # BUG1
    # ======================================================

    def start_boundary_follow(self):

        self.hit_point = (
            self.x,
            self.y
        )

        self.leave_point = (
            self.x,
            self.y
        )

        self.best_goal_dist = self._dist_to_goal()

        self.travelled_on_boundary = 0.0
        self.boundary_completed = False

        self.state = FOLLOW_BOUNDARY

        self._cancel_pid()

    def update_leave_point(self):

        d = self._dist_to_goal()

        if d < self.best_goal_dist:

            self.best_goal_dist = d

            self.leave_point = (
                self.x,
                self.y
            )

    def reached_hit_point(self):

        if self.hit_point is None:
            return False

        return (
            self.distance(
                (self.x, self.y),
                self.hit_point
            ) < self.hit_radius
            and
            self.travelled_on_boundary >
            self.min_boundary_length
        )

    # ======================================================
    # CONTROL LOOP
    # ======================================================

    def control_loop(self):

        if self.state == GOAL_REACHED:
            self._cmd(0.0, 0.0)
            return

        if not self.scan_ready:
            return

        dist_goal = self._dist_to_goal()

        if dist_goal < self.goal_tolerance:

            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1} alcanzado'
            )

            self.advance_to_next_waypoint()

            return

        if self.state == GO_TO_GOAL:

            if self.scan_front < self.obstacle_threshold:

                self.get_logger().info(
                    'Hit Point detectado'
                )

                self.start_boundary_follow()

                return

            if not self._pid_goal_sent:

                self._send_setpoint(
                    self.goal_x,
                    self.goal_y
                )

                self._pid_goal_sent = True

        elif self.state == FOLLOW_BOUNDARY:

            self.update_leave_point()

            if self.reached_hit_point():

                self.boundary_completed = True

                self.state = RETURN_TO_LEAVE

                self.get_logger().info(
                    'Perímetro completado'
                )

                return

            self.wall_follow()

        elif self.state == RETURN_TO_LEAVE:

            if self.distance(
                (self.x, self.y),
                self.leave_point
            ) < self.leave_radius:

                self.hit_point = None
                self.leave_point = None
                self.boundary_completed = False
                self.travelled_on_boundary = 0.0

                self.state = GO_TO_GOAL
                self._pid_goal_sent = False

                self.get_logger().info(
                    'Leave Point alcanzado'
                )

                return

            self.wall_follow()

    # ======================================================

    def wall_follow(self):

        front_free = (
            self.scan_front >=
            self.obstacle_threshold
        )

        wall_error = (
            self.scan_right -
            self.wall_dist_target
        )

        angular = np.clip(
            -self.kp_wall * wall_error,
            -self.wall_angular_speed,
            self.wall_angular_speed
        )

        if not front_free:

            self._cmd(
                0.0,
                self.wall_angular_speed
            )

        else:

            self._cmd(
                self.wall_linear_speed,
                angular
            )
    
    def advance_to_next_waypoint(self):

        if self.waypoint_index < len(self.waypoints) - 1:

            self.waypoint_index += 1

            self._pid_goal_sent = False

            self.get_logger().info(
                f'Siguiente waypoint: '
                f'{self.waypoint_index + 1}'
            )

            self.state = GO_TO_GOAL

        else:

            self.state = GOAL_REACHED

            self._cancel_pid()

            self.get_logger().info(
                'Trayectoria completada'
            )

    # ======================================================

    def debug_log(self):

        if len(self.waypoints) == 0:

            self.get_logger().warn(
                'No hay waypoints cargados'
            )
            return

        self.get_logger().info(
            f'State={self.state} | '
            f'WP={self.waypoint_index + 1}/{len(self.waypoints)} | '
            f'Goal=({self.goal_x:.2f},{self.goal_y:.2f}) | '
            f'd_goal={self._dist_to_goal():.2f} | '
            f'boundary={self.travelled_on_boundary:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Bug1Node()

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