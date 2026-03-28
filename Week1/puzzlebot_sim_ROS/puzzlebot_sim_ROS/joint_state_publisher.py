import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import transforms3d
import numpy as np

class PuzzlebotPublisher(Node):

    def __init__(self):
        super().__init__('frame_publisher')

        #Drone Initial Pose
        self.intial_pos_x = 1.0
        self.intial_pos_y = 1.0
        self.intial_pos_z = 1.0
        self.intial_pos_yaw = np.pi/2
        self.intial_pos_pitch = 0.0
        self.intial_pos_roll = 0.0

        #Angular velocity for the pose change and wheels
        self.omega = 0.5
        self.omega_wheel = 100.0

        #Define Transformations
        self.define_TF()

        #Define Markers
        self.define_markers()

        #Create Transform Broadcasters
        self.tf_br_base = TransformBroadcaster(self)
        self.tf_br_base_footprint = TransformBroadcaster(self)
        self.tf_br_wheelr = TransformBroadcaster(self)
        self.tf_br_wheell = TransformBroadcaster(self)
        self.tf_br_caster = TransformBroadcaster(self)

        #Create Markers Publishers
        self.base_marker_pub = self.create_publisher(Marker, '/base_marker', 10)
        self.wheelr_marker_pub = self.create_publisher(Marker, '/wheelr_marker', 10)
        self.wheell_marker_pub = self.create_publisher(Marker, '/wheell_marker', 10)
        self.caster_marker_pub = self.create_publisher(Marker, '/caster_marker', 10)

        #Create a Timer
        timer_period = 0.005 #seconds
        self.timer = self.create_timer(timer_period, self.timer_cb)


    #Timer Callback
    def timer_cb(self):

        time = self.get_clock().now().nanoseconds/1e9

        self.base.header.stamp = self.get_clock().now().to_msg()
        self.wheelr.header.stamp = self.get_clock().now().to_msg()
        self.wheell.header.stamp = self.get_clock().now().to_msg()
        self.caster.header.stamp = self.get_clock().now().to_msg()

        #Create Trasnform Messages
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_link_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw+self.omega*time)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]

        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x + 0.5*np.cos(self.omega*time)
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y + 0.5*np.sin(self.omega*time)
        self.base_footprint_tf.transform.translation.z = 0.0
        q = transforms3d.euler.euler2quat(0, 0, self.intial_pos_yaw+self.omega*time)       
        self.base_footprint_tf.transform.rotation.x = q[1]
        self.base_footprint_tf.transform.rotation.y = q[2]
        self.base_footprint_tf.transform.rotation.z = q[3]
        self.base_footprint_tf.transform.rotation.w = q[0]

        self.wheel_r_tf.header.stamp = self.get_clock().now().to_msg()
        q_wheelr = transforms3d.euler.euler2quat(0, 0, self.omega_wheel*time)       
        self.wheel_r_tf.transform.rotation.x = q_wheelr[1]
        self.wheel_r_tf.transform.rotation.y = q_wheelr[2]
        self.wheel_r_tf.transform.rotation.z = q_wheelr[3]
        self.wheel_r_tf.transform.rotation.w = q_wheelr[0]

        self.wheel_l_tf.header.stamp = self.get_clock().now().to_msg()
        q_wheell = transforms3d.euler.euler2quat(0, 0, self.omega_wheel*time)       
        self.wheel_l_tf.transform.rotation.x = q_wheell[1]
        self.wheel_l_tf.transform.rotation.y = q_wheell[2]
        self.wheel_l_tf.transform.rotation.z = q_wheell[3]
        self.wheel_l_tf.transform.rotation.w = q_wheell[0]

        self.caster_tf.header.stamp = self.get_clock().now().to_msg()
        q_caster = transforms3d.euler.euler2quat(0, 0, self.omega_wheel*time)       
        self.caster_tf.transform.rotation.x = q_caster[1]
        self.caster_tf.transform.rotation.y = q_caster[2]
        self.caster_tf.transform.rotation.z = q_caster[3]
        self.caster_tf.transform.rotation.w = q_caster[0]

        self.tf_br_base.sendTransform(self.base_link_tf)
        self.tf_br_base_footprint.sendTransform(self.base_footprint_tf)
        self.tf_br_wheelr.sendTransform(self.wheel_r_tf)
        self.tf_br_wheell.sendTransform(self.wheel_l_tf)
        self.tf_br_caster.sendTransform(self.caster_tf)

        self.base_marker_pub.publish(self.base)
        self.wheelr_marker_pub.publish(self.wheelr)
        self.wheell_marker_pub.publish(self.wheell)
        self.caster_marker_pub.publish(self.caster)  

    def define_markers(self):

        #initialise the marker(the pose and orientation will be changed on the callback function)
        self.base = Marker()
        self.base.header.frame_id = "base_link"
        self.base.header.stamp = self.get_clock().now().to_msg()
        self.base.id = 0
        self.base.type = Marker.MESH_RESOURCE
        self.base.mesh_resource = "package://puzzlebot_sim_ROS/meshes/Puzzlebot_Jetson_Lidar_Edition_Base.stl"
        self.base.action = Marker.ADD
        self.base.pose.position.x = 0.0
        self.base.pose.position.y = 0.0
        self.base.pose.position.z = 0.05
        q_base_marker = transforms3d.euler.euler2quat(1.57, 0.0, 1.57) 
        self.base.pose.orientation.x = q_base_marker[1]
        self.base.pose.orientation.y = q_base_marker[2]
        self.base.pose.orientation.z = q_base_marker[3]
        self.base.pose.orientation.w = q_base_marker[0]
        self.base.scale.x = 1.0
        self.base.scale.y = 1.0
        self.base.scale.z = 1.0
        self.base.color.r = 1.0
        self.base.color.g = 1.0
        self.base.color.b = 0.0

        self.wheelr = Marker()
        self.wheelr.header.frame_id = "wheelr"
        self.wheelr.header.stamp = self.get_clock().now().to_msg()
        self.wheelr.id = 0
        self.wheelr.type = Marker.MESH_RESOURCE
        self.wheelr.mesh_resource = "package://puzzlebot_sim_ROS/meshes/Puzzlebot_Wheel.stl"
        self.wheelr.action = Marker.ADD
        self.wheelr.pose.position.x = 0.052 
        self.wheelr.pose.position.y = -0.095
        self.wheelr.pose.position.z = -0.0025
        self.wheelr.pose.orientation.x = 0.0
        self.wheelr.pose.orientation.y = 0.0
        self.wheelr.pose.orientation.z = 0.0
        self.wheelr.pose.orientation.w = 1.0
        self.wheelr.scale.x = 1.0
        self.wheelr.scale.y = 1.0
        self.wheelr.scale.z = 1.0
        self.wheelr.color.r = 1.0
        self.wheelr.color.g = 1.0
        self.wheelr.color.b = 0.0
        self.wheelr.color.a = 1.0

        self.wheell = Marker()
        self.wheell.header.frame_id = "wheell"
        self.wheell.header.stamp = self.get_clock().now().to_msg()
        self.wheell.id = 0
        self.wheell.type = Marker.MESH_RESOURCE
        self.wheell.mesh_resource = "package://puzzlebot_sim_ROS/meshes/Puzzlebot_Wheel.stl"
        self.wheell.action = Marker.ADD
        self.wheell.pose.position.x = 0.052 
        self.wheell.pose.position.y = 0.095
        self.wheell.pose.position.z = -0.0025
        self.wheell.pose.orientation.x = 0.0
        self.wheell.pose.orientation.y = 0.0
        self.wheell.pose.orientation.z = 0.0
        self.wheell.pose.orientation.w = 1.0
        self.wheell.scale.x = 1.0
        self.wheell.scale.y = 1.0
        self.wheell.scale.z = 1.0
        self.wheell.color.r = 1.0
        self.wheell.color.g = 1.0
        self.wheell.color.b = 0.0
        self.wheell.color.a = 1.0

        self.caster = Marker()
        self.caster.header.frame_id = "caster"
        self.caster.header.stamp = self.get_clock().now().to_msg()
        self.caster.id = 0
        self.caster.type = Marker.MESH_RESOURCE
        self.caster.mesh_resource = "package://puzzlebot_sim_ROS/meshes/Puzzlebot_Caster_Wheel.stl"
        self.caster.action = Marker.ADD
        self.caster.pose.position.x = -0.095 
        self.caster.pose.position.y = 0.0
        self.caster.pose.position.z = -0.03
        self.caster.pose.orientation.x = 0.0
        self.caster.pose.orientation.y = 0.0
        self.caster.pose.orientation.z = 0.0
        self.caster.pose.orientation.w = 1.0
        self.caster.scale.x = 1.0
        self.caster.scale.y = 1.0
        self.caster.scale.z = 1.0
        self.caster.color.r = 1.0
        self.caster.color.g = 1.0
        self.caster.color.b = 0.0
        self.caster.color.a = 1.0

    def define_TF(self):

        #Create Trasnform Messages
        self.base_footprint_tf = TransformStamped()
        self.base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_footprint_tf.header.frame_id = 'odom'
        self.base_footprint_tf.child_frame_id = 'base_footprint'
        self.base_footprint_tf.transform.translation.x = self.intial_pos_x
        self.base_footprint_tf.transform.translation.y = self.intial_pos_y
        self.base_footprint_tf.transform.translation.z = 0.0
        q_foot = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_footprint_tf.transform.rotation.x = q_foot[1]
        self.base_footprint_tf.transform.rotation.y = q_foot[2]
        self.base_footprint_tf.transform.rotation.z = q_foot[3]
        self.base_footprint_tf.transform.rotation.w = q_foot[0]


        #Create Trasnform Messages
        self.base_link_tf = TransformStamped()
        self.base_link_tf.header.stamp = self.get_clock().now().to_msg()
        self.base_link_tf.header.frame_id = 'base_footprint'
        self.base_link_tf.child_frame_id = 'base_link'
        self.base_link_tf.transform.translation.x = self.intial_pos_x
        self.base_link_tf.transform.translation.y = self.intial_pos_y
        self.base_link_tf.transform.translation.z = self.intial_pos_z
        q = transforms3d.euler.euler2quat(self.intial_pos_roll, self.intial_pos_pitch, self.intial_pos_yaw)       
        self.base_link_tf.transform.rotation.x = q[1]
        self.base_link_tf.transform.rotation.y = q[2]
        self.base_link_tf.transform.rotation.z = q[3]
        self.base_link_tf.transform.rotation.w = q[0]


        self.wheel_r_tf = TransformStamped()
        #Create Trasnform Messages
        self.wheel_r_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_r_tf.header.frame_id = 'base_link'
        self.wheel_r_tf.child_frame_id = 'wheel_r'
        self.wheel_r_tf.transform.translation.x = 0.06717
        self.wheel_r_tf.transform.translation.y = 0.082
        self.wheel_r_tf.transform.translation.z = -0.0125
        q_wheelr = transforms3d.euler.euler2quat(0, 0, 0)       
        self.wheel_r_tf.transform.rotation.x = q_wheelr[1]
        self.wheel_r_tf.transform.rotation.y = q_wheelr[2]
        self.wheel_r_tf.transform.rotation.z = q_wheelr[3]
        self.wheel_r_tf.transform.rotation.w = q_wheelr[0]


        self.wheel_l_tf = TransformStamped()
        self.wheel_l_tf.header.stamp = self.get_clock().now().to_msg()
        self.wheel_l_tf.header.frame_id = 'base_link'
        self.wheel_l_tf.child_frame_id = 'wheel_l'
        self.wheel_l_tf.transform.translation.x = -0.06717
        self.wheel_l_tf.transform.translation.y = 0.082
        self.wheel_l_tf.transform.translation.z = -0.0125
        q_wheell = transforms3d.euler.euler2quat(0, 0, 0)       
        self.wheel_l_tf.transform.rotation.x = q_wheell[1]
        self.wheel_l_tf.transform.rotation.y = q_wheell[2]
        self.wheel_l_tf.transform.rotation.z = q_wheell[3]
        self.wheel_l_tf.transform.rotation.w = q_wheell[0]

        self.caster_tf = TransformStamped()
        self.caster_tf.header.stamp = self.get_clock().now().to_msg()
        self.caster_tf.header.frame_id = 'base_link'
        self.caster_tf.child_frame_id = 'caster'
        self.caster_tf.transform.translation.x = -0.06717
        self.caster_tf.transform.translation.y = -0.082
        self.caster_tf.transform.translation.z = -0.0125
        q_caster = transforms3d.euler.euler2quat(0, 0, 0)       
        self.caster_tf.transform.rotation.x = q_caster[1]
        self.caster_tf.transform.rotation.y = q_caster[2]
        self.caster_tf.transform.rotation.z = q_caster[3]
        self.caster_tf.transform.rotation.w = q_caster[0]

def main(args=None):
    rclpy.init(args=args)

    node = PuzzlebotPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():  # Ensure shutdown is only called once
            rclpy.shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()