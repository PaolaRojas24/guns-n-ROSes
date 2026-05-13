import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
import math
import csv
import os


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger_node')

        self.declare_parameter('output_file', os.path.expanduser('~/calibracion_datos.csv'))
        self.declare_parameter('num_waypoints', 2)

        self.output_file = self.get_parameter('output_file').value
        self.num_waypoints = self.get_parameter('num_waypoints').value

        self.current_pose = None
        self.waypoints_reached = 0
        self.run_done = False

        self.run_number = self._count_existing_runs() + 1

        self.create_subscription(Pose, 'pose', self.pose_cb, 10)
        self.create_subscription(Point, 'next_point', self.next_point_cb, 10)

        self.get_logger().info(
            f'DataLogger listo — corrida #{self.run_number}, '
            f'waypoints esperados: {self.num_waypoints}, '
            f'archivo: {self.output_file}'
        )

    def _count_existing_runs(self):
        if not os.path.exists(self.output_file):
            return 0
        with open(self.output_file, 'r') as f:
            lines = [l for l in f.readlines() if l.strip() and not l.startswith('run')]
        return len(lines)

    def pose_cb(self, msg: Pose):
        self.current_pose = msg

    def next_point_cb(self, msg: Point):
        if self.run_done:
            return
        self.waypoints_reached += 1
        self.get_logger().info(
            f'Waypoint {self.waypoints_reached}/{self.num_waypoints} alcanzado'
        )
        if self.waypoints_reached >= self.num_waypoints:
            self.run_done = True
            self._save_final_pose()

    def _save_final_pose(self):
        if self.current_pose is None:
            self.get_logger().error('Sin datos de pose al momento de guardar.')
            return

        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # puzzlebot_sim suma math.pi a todos los componentes del cuaternion,
        # por eso restamos pi antes de aplicar atan2
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        yaw = 2.0 * math.atan2(qz - math.pi, qw - math.pi)
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))  # normalizar a [-pi, pi]

        file_exists = os.path.exists(self.output_file)
        with open(self.output_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(['run', 'x_final', 'y_final', 'theta_final_rad'])
            writer.writerow([
                self.run_number,
                round(x, 6),
                round(y, 6),
                round(yaw, 6)
            ])

        self.get_logger().info(
            f'[Corrida #{self.run_number}] GUARDADA → '
            f'x={x:.4f} m, y={y:.4f} m, θ={math.degrees(yaw):.2f}° '
            f'| {self.output_file}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
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
