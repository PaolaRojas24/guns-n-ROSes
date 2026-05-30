"""Mission node — secuencia waypoints y manda un goal a la vez al planner."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


_N_GOALS = 4


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        for i in range(1, _N_GOALS + 1):
            self.declare_parameter(f'trajectory_goals.goal_{i}.x', 0.0)
            self.declare_parameter(f'trajectory_goals.goal_{i}.y', 0.0)

        self.declare_parameter('start_delay_s',  5.0)
        self.declare_parameter('loop_waypoints', True)

        self.start_delay_s  = self.get_parameter('start_delay_s').value
        self.loop_waypoints = self.get_parameter('loop_waypoints').value

        self.waypoints = self._load_waypoints()
        self.idx       = 0
        self.started   = False

        if not self.waypoints:
            self.get_logger().error('No se cargaron waypoints en el YAML del mundo.')
        else:
            self.get_logger().info(
                f'Trayectoria cargada ({len(self.waypoints)} waypoints):'
            )
            for i, (wx, wy) in enumerate(self.waypoints):
                self.get_logger().info(f'  goal_{i + 1}: ({wx:.2f}, {wy:.2f})')

        # /goal_pose con TRANSIENT_LOCAL para que late-joiners (planner, controller) lo reciban
        qos_goal = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', qos_goal)
        self.pub_viz  = self.create_publisher(MarkerArray, '/mission/viz', 10)

        self.create_subscription(Bool, '/goal_reached', self.goal_reached_cb, 10)

        # Espera a que SLAM publique /map y los nodos se conecten
        self.start_timer = self.create_timer(self.start_delay_s, self._start_mission)
        self.create_timer(1.0, self._publish_viz)

    def _load_waypoints(self):
        wps = []
        for i in range(1, _N_GOALS + 1):
            x = self.get_parameter(f'trajectory_goals.goal_{i}.x').value
            y = self.get_parameter(f'trajectory_goals.goal_{i}.y').value
            if x == 0.0 and y == 0.0:
                continue
            wps.append((float(x), float(y)))
        return wps

    def _start_mission(self):
        self.start_timer.cancel()
        if not self.waypoints:
            return
        self.started = True
        self.get_logger().info('Misión iniciada')
        self._send_current_goal()

    def _send_current_goal(self):
        wx, wy = self.waypoints[self.idx]
        msg = PoseStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.position.x = wx
        msg.pose.position.y = wy
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.get_logger().info(
            f'Goal activo: goal_{self.idx + 1} ({wx:.2f}, {wy:.2f})'
        )

    def goal_reached_cb(self, msg: Bool):
        if not msg.data or not self.started:
            return
        self.get_logger().info(f'goal_{self.idx + 1} alcanzado')
        nxt = self.idx + 1
        if nxt >= len(self.waypoints):
            if self.loop_waypoints:
                self.idx = 0
            else:
                self.get_logger().info('Misión completa')
                self.started = False
                return
        else:
            self.idx = nxt
        self._send_current_goal()

    def _publish_viz(self):
        if not self.waypoints:
            return
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for i, (wx, wy) in enumerate(self.waypoints):
            active = (i == self.idx and self.started)
            cyl = Marker()
            cyl.header.frame_id = 'world'
            cyl.header.stamp    = stamp
            cyl.ns              = 'waypoints'
            cyl.id              = i
            cyl.type            = Marker.CYLINDER
            cyl.action          = Marker.ADD
            cyl.pose.position.x = wx
            cyl.pose.position.y = wy
            cyl.pose.position.z = 0.05
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = cyl.scale.y = 0.25
            cyl.scale.z = 0.10
            cyl.color = ColorRGBA(
                r=1.0, g=(1.0 if active else 0.6), b=0.0,
                a=(0.9 if active else 0.4)
            )
            arr.markers.append(cyl)

            txt = Marker()
            txt.header.frame_id = 'world'
            txt.header.stamp    = stamp
            txt.ns              = 'waypoint_labels'
            txt.id              = i
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position.x = wx
            txt.pose.position.y = wy
            txt.pose.position.z = 0.35
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.18
            txt.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text    = f'goal_{i + 1}'
            arr.markers.append(txt)
        self.pub_viz.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
