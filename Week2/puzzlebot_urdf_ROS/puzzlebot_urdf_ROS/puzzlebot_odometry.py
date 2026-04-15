import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math


class PuzzlebotOdometry(Node):

    def __init__(self):
        super().__init__('puzzlebot_odometry')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('wheel_radius', 0.05)  # m
        self.declare_parameter('wheel_base',   0.18)  # m (L, distance between wheels)

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_base').value

        # ── State ─────────────────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        self.vr = 0.0   # right wheel rad/s
        self.vl = 0.0   # left  wheel rad/s

        self.prev_time = None

        # ── TF broadcaster ────────────────────────────────────────────────────
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ── Publishers ────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)

        self.get_logger().info(
            f'Odometry node started — r={self.r} m, L={self.L} m'
        )

    # ── Joint states callback ─────────────────────────────────────────────────
    def joint_states_cb(self, msg: JointState):
        """Compute and publish odometry from wheel joint states."""

        # ── Ensure required joints exist ─────────────────────────────────────────
        try:
            idx_r = msg.name.index('base_link_to_wheel_r')
            idx_l = msg.name.index('base_link_to_wheel_l')
        except ValueError:
            self.get_logger().warn(
                'Expected joint names not found in /joint_states'
            )
            return

        # ── Ensure velocity data is available ────────────────────────────────────
        if len(msg.velocity) <= max(idx_r, idx_l):
            self.get_logger().warn(
                'Velocity data missing in /joint_states'
            )
            return

        # Extract wheel angular velocities (rad/s)
        self.vr = msg.velocity[idx_r]
        self.vl = -msg.velocity[idx_l]

        # ── Time handling ────────────────────────────────────────────────────────
        now = self.get_clock().now()

        # Initialize time on the first callback
        if self.prev_time is None:
            self.prev_time = now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        # Prevent invalid integration
        if dt <= 0.0:
            return

        # ── Differential-drive kinematics ────────────────────────────────────────
        v_robot = self.r * -(self.vr + self.vl) / 2.0
        w_robot = self.r * -(self.vl - self.vr) / self.L

        # Update pose
        self.x += v_robot * math.cos(self.yaw) * dt
        self.y += v_robot * math.sin(self.yaw) * dt
        self.yaw += w_robot * dt

        # Normalize yaw to [-π, π]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ── Convert yaw to quaternion ────────────────────────────────────────────
        yaw_corrected = self.yaw + math.pi / 2.0

        # Convert to quaternion
        qz = math.sin(yaw_corrected / 2.0)
        qw = math.cos(yaw_corrected / 2.0)

        # ── Publish odom → base_footprint TF ─────────────────────────────────────
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # ── Publish Odometry message ─────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist
        odom.twist.twist.linear.x = v_robot
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w_robot

        self.odom_pub.publish(odom)

# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotOdometry()
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