import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitude

from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)

from Vehicle import Vehicle


class OffboardControl(Node):

    def __init__(self):
        super().__init__('OffboardControl')
        self.subscribers = []
        self.drone = Vehicle()
        self.drone2 = Vehicle(ros_namespace="drone2", node=self)

        if not self.drone.is_armed():
            self.drone.send_arm_command()

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()