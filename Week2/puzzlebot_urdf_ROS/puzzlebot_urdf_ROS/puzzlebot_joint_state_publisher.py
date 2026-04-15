import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PuzzlebotJointStatePublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot_joint_state_publisher')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('omega_wheel', 1.0)   # rad/s — wheel spin speed
        self.declare_parameter('timer_period', 0.05) # seconds — publish rate

        self.omega_wheel  = self.get_parameter('omega_wheel').value
        timer_period      = self.get_parameter('timer_period').value

        # ── State ─────────────────────────────────────────────────────────────
        self.angle_r = 0.0   # accumulated angle for right wheel (rad)
        self.angle_l = 0.0   # accumulated angle for left  wheel (rad)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.timer = self.create_timer(timer_period, self.timer_cb)
        self.get_logger().info(
            f'Joint state publisher started — omega_wheel={self.omega_wheel} rad/s'
        )

    # ── Timer callback ────────────────────────────────────────────────────────
    def timer_cb(self):

        dt = self.timer.timer_period_ns / 1e9  

        # Integrate angle
        self.angle_r += self.omega_wheel * dt
        self.angle_l += self.omega_wheel * dt

        # Build message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name     = ['base_link_to_wheel_r', 'base_link_to_wheel_l']
        msg.position = [self.angle_r, self.angle_l]
        msg.velocity = [self.omega_wheel, self.omega_wheel]
        msg.effort   = []

        self.pub.publish(msg)


# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotJointStatePublisher()
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
