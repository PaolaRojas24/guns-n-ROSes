# Joint state Publisher for the Puzzlebot - guns-n-ROSes
# Uses the URDF to get the wheel links and publishes the Wheels states.
# Recives as parameters the time and  omega from each wheel.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PuzzlebotJointStatePublisher(Node):

    def __init__(self):
        super().__init__('puzzlebot_joint_state_publisher')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('omega_wheel_r', 1.0)   # rad/s 
        self.declare_parameter('omega_wheel_l', 1.0)   # rad/s 
        self.declare_parameter('timer_period', 0.05)   # seconds

        self.omega_wheel_r  = self.get_parameter('omega_wheel_r').value
        self.omega_wheel_l  = self.get_parameter('omega_wheel_l').value
        timer_period      = self.get_parameter('timer_period').value

        # ── State ─────────────────────────────────────────────────────────────
        # Accumulated angles
        self.angle_r = 0.0   
        self.angle_l = 0.0  

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.timer = self.create_timer(timer_period, self.timer_cb)

    def timer_cb(self):

        dt = self.timer.timer_period_ns / 1e9  

        # Integrate the angle
        self.angle_r += self.omega_wheel_r * dt
        self.angle_l += self.omega_wheel_l * dt

        # Build the message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name     = ['base_link_to_wheel_r', 'base_link_to_wheel_l']
        msg.position = [self.angle_r, self.angle_l]
        msg.velocity = [self.omega_wheel_r, self.omega_wheel_l]
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
