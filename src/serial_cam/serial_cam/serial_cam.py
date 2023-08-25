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
        if ros_namespace == '/':
            ros_namespace = ""

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.camera_param_pub = self.create_publisher(CameraInfo, self.ros_namespace+"/camera/camera_info", 10)
        self.marker_info_pub = self.create_publisher(Float64MultiArray, self.ros_namespace+"/camera/marker_pos", 10)
        
        ## Publish camera reference frame
        t = TransformStamped()
        q = quaternion_from_euler(3.1415, 3.1415, 0.0) 
        t.header.stamp = self.get_clock().now()
        t.header.frame_id = self.ros_namespace
        t.child_frame_id = self.ros_namespace + "/camera"
        t.transform.translation.x = 0.7
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.static_broadcaster.sendTransform(t)

        ## Publish camera intrinsic params
        info = CameraInfo()
        info.d = [-0.40435636, 0.05816364 , 0.00363366 , 0.00228936 , 0.1716887 ]
        info.k = [153.75337327, 0.0e+00, 77.20143553, 
			0.0e+00, 153.95236529, 58.43806698,     
			0.0e+00, 0.0e+00, 1.0e+00 ]
        self.camera_param_pub.publish(info)

        self.read_serial()


    def read_serial(self):
        print("Initialising serial port")
        with serial.Serial('/dev/ttyUSB0', 115200) as ser:
            ser.flush()
            while(True):
                line = ser.readline()
                line = line.decode('utf-8')
                print(f"Publishing {line}")
                fields = line.split(",")
                try:
                    tag_id = float(fields[0])
                    tag_cx = float(fields[1])
                    tag_cy = float(fields[2])

                    new_msg = Float64MultiArray()
                    new_msg.data.append(tag_id)
                    new_msg.data.append(tag_cx)
                    new_msg.data.append(tag_cy)
                    self.marker_info_pub.publish(new_msg)
                except:
                    pass
        
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
