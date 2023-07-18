import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)
from rclpy.clock import Clock

import serial
from std_msgs.msg import Float64MultiArray

class SerialCamera(Node) :
    def __init__(self) -> None:
        super().__init__('camerasim_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        ros_namespace = self.get_namespace()
        if ros_namespace == '/':
            ros_namespace = ""

        self.marker_info_pub = self.create_publisher(Float64MultiArray, ros_namespace+"/openmv_serial", 10)
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
        


def main(args=None):
    rclpy.init(args=args)
    sub = SerialCamera()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
