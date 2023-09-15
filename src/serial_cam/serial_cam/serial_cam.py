import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)

import math

from tf2_ros import StaticTransformBroadcaster

import serial
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

class SerialCamera(Node) :
    def __init__(self) -> None:
        super().__init__('camerasim_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.ros_namespace = self.get_namespace()
        if self.ros_namespace == '/':
            self.ros_namespace = ""

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.camera_param_pub = self.create_publisher(CameraInfo, self.ros_namespace+"/camera/camera_info", 10)
        self.marker_info_pub = self.create_publisher(Float64MultiArray, self.ros_namespace+"/camera/marker_pos", 10)
        self.serial_timer = self.create_timer(0.06, self.timer_cb)
        
        ## Publish camera reference frame
        t = TransformStamped()
        # q = quaternion_from_euler(0.0, 3.1415, 3.1415) 
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ros_namespace[1:]
        t.child_frame_id = self.ros_namespace[1:] + "/camera"
        t.transform.translation.x = 0.07
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0 #q[0]
        t.transform.rotation.y = 0.0 #q[1]
        t.transform.rotation.z = 1.0 #q[2]
        t.transform.rotation.w = 0.0 #q[3]
        # self.static_broadcaster.sendTransform(t)

        ## Publish camera intrinsic params
        self.info = CameraInfo()
        self.info.d = [-0.40435636, 0.05816364 , 0.00363366 , 0.00228936 , 0.1716887 ]
        self.info.k = [153.75337327, 0.0e+00, 77.20143553, 
			0.0e+00, 153.95236529, 58.43806698,     
			0.0e+00, 0.0e+00, 1.0e+00 ]
        self.camera_param_pub.publish(self.info)

        print("Initialising serial port")
        self.serial_dev = serial.Serial('/dev/ttyS2', 115200)
        self.serial_dev.flush()


    def timer_cb(self):
        line = self.serial_dev.readline()
        line = line.decode('utf-8')
        print(f"Publishing {line}")
        fields = line.split(",")
        try:
            tag_id = float(fields[0])
            tag_cx = float(fields[1])
            tag_cy = float(fields[2])
        
            if tag_id == 2:
                new_msg = Float64MultiArray()
                new_msg.data.append(tag_id)
                new_msg.data.append(tag_cx)
                new_msg.data.append(tag_cy)
                self.marker_info_pub.publish(new_msg)
                self.camera_param_pub.publish(self.info) 
        except Exception as e:
            self.get_logger().info(e)
        
        
def quaternion_from_euler(roll, pitch, yaw) -> list:
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def main(args=None):
    rclpy.init(args=args)
    sub = SerialCamera()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

