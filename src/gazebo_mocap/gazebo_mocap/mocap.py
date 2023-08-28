import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)
from rclpy.clock import Clock

from nav_msgs.msg import (
    Path,
    Odometry)
from geometry_msgs.msg import (
    Pose,
    PoseStamped)
from px4_msgs.msg import VehicleOdometry

from math import sqrt
import numpy as np



class MocapNode (Node):
    def __init__(self) -> None:
        super().__init__('mocap_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.ros_ns = self.get_namespace()
        if self.ros_ns == "/":
            self.ros_ns = ""
        # Subscribers
        self.subscriber_ =      self.create_subscription(Odometry, self.ros_ns+"/odom", self.listener_cb, 10)
        # Publishers
        self.mocap_odom_pub =   self.create_publisher(VehicleOdometry,  self.ros_ns+"/fmu/in/vehicle_visual_odometry", 10)
        # Messages
        self.vehicle_path_msg = Path()
        self.vehicle_path_msg.header.frame_id = 'map'
        self.vehicle_pose_msg = PoseStamped()
        self.gz_pose_msg = Pose()
        self.mocap_odom_msg = None

        self.path_cnt = 0

    def listener_cb(self, msg: Odometry):
        self.gz_pose_msg : Pose = msg.pose.pose

        # Change ref fram to FLU and publish msg
        self.vehicle_pose_msg = self.gazebo_to_flu_tf(self.gz_pose_msg)
        # Create mocap odometry msg and publish to PX4
        self.mocap_odom_msg = self.create_odometry_msg(self.vehicle_pose_msg.pose)
        self.mocap_odom_pub.publish(self.mocap_odom_msg)


    def gazebo_to_flu_tf(self, gz_pose: Pose) -> PoseStamped:
        """
        Transform from gazebo Pose to FLU pose for ROS2. Is done doing
        a rotation of -90° arounf z axis (global frame). 
        """
        vehicle_pose_msg = PoseStamped()
        vehicle_pose_msg.header.frame_id='map'
        # Position
        vehicle_pose_msg.pose.position = gz_pose.position
        # Orientation
        vehicle_pose_msg.pose.orientation = gz_pose.orientation
        return vehicle_pose_msg
    

    def flu_to_ned_tf(self, pose : Pose) -> Pose:
        """
        Transform a FLU frame into NED => Rot_z(90°)
        """
        pose_quat = (pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z)
        rot_quat = (0.0, sqrt(2)/2, 0.0, sqrt(2)/2)
        result = self.quaternion_multiply(pose_quat, rot_quat)

        ned_pose = Pose()
        ned_pose.position.x =  pose.position.x
        ned_pose.position.y = -pose.position.y
        ned_pose.position.z = -pose.position.z
        ned_pose.orientation.x = result[1] 
        ned_pose.orientation.y = result[2] 
        ned_pose.orientation.z = result[3] 
        ned_pose.orientation.w = result[0] 
        return ned_pose

    
    def create_odometry_msg(self, pose : Pose) -> VehicleOdometry:
        """
        Create VehicleOdometry message. Not include velocities
        """
        odom_msg = VehicleOdometry()
        odom_msg.timestamp =  int(Clock().now().nanoseconds / 1000)
        odom_msg.timestamp_sample = int(Clock().now().nanoseconds / 1000)
        odom_msg.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        ned_pose = self.flu_to_ned_tf(pose)
        odom_msg.position = [ned_pose.position.x, 
                            ned_pose.position.y, 
                            ned_pose.position.z]
        odom_msg.q =        [ned_pose.orientation.x,
                            ned_pose.orientation.y,                      
                            ned_pose.orientation.z,
                            ned_pose.orientation.w] 
        odom_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        odom_msg.velocity =         [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.angular_velocity = [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.position_variance =    [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.orientation_variance = [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.velocity_variance =    [float('NaN'), float('Nan'), float('Nan')]
        odom_msg.quality = 1
        return odom_msg
    

    def quaternion_multiply(self, quaternion1, quaternion0) -> np.ndarray:
        """
        Multiply two quaternions in the form (w, x, y, z)
        """
        w0, x0, y0, z0 = quaternion0
        w1, x1, y1, z1 = quaternion1
        return np.array([-x1*x0 - y1*y0 - z1*z0 + w1*w0,
                            x1*w0 + y1*z0 - z1*y0 + w1*x0,
                            -x1*z0 + y1*w0 + z1*x0 + w1*y0,
                            x1*y0 - y1*x0 + z1*w0 + w1*z0], dtype=np.float64)

def main(args=None):
    rclpy.init(args=args)
    sub = MocapNode()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
