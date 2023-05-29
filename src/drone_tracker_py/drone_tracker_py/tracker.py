import rclpy
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import utilities
import subprocess

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

from .Vehicle import Vehicle
from .Pilot import Pilot


class DroneTracker(Node):

    def __init__(self):
        super().__init__('DroneTracker')

        self.declare_parameter("namespace", "drone1")
        self.declare_parameter("simulation", True)
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value

        self.drone = Vehicle(self, self.namespace)
        self.pilot = Pilot()

        self.pilot.set_vehicle(self.drone)
        self.offboard_timer = self.create_timer(0.1, self.pilot.offboard_callback)
        self.mission_timer = self.create_timer(0.5, self.pilot.mission_update)

def main(args=None):
    rclpy.init(args=args)
    print("Starting offboard control node...\n")
    offboard_control = DroneTracker()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()