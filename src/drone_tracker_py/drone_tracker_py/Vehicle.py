import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
import copy
from .CameraTransform import CameraTransform

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import ApriltagMarker

from sensor_msgs.msg import Image

from rclpy.qos import (
    QoSProfile, 
    QoSReliabilityPolicy, 
    QoSHistoryPolicy)


class Vehicle():
# Class representing the vehicle. Subscribe to PX4 topics to update its state

    def __init__(self, node:Node, ros_namespace:str="") -> None:

        self._subscribers = []
        self._publishers = {}
        self._attached_node = node
        self.namespace = ros_namespace
        self.camera = CameraTransform()

        self._qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._pub_topics:list = [
            (VehicleCommand, '/' + ros_namespace + "/fmu/in/vehicle_command"),
            (OffboardControlMode, '/' + ros_namespace + "/fmu/in/offboard_control_mode"),
            (TrajectorySetpoint, '/' + ros_namespace + "/fmu/in/trajectory_setpoint")
        ]

        self._sub_topics:list = [
            (VehicleStatus, '/' + ros_namespace + "/fmu/out/vehicle_status"),
            (VehicleAttitude, '/' + ros_namespace + "/fmu/out/vehicle_attitude"),
            (VehicleControlMode, '/' + ros_namespace + "/fmu/out/vehicle_control_mode"),
            (TimesyncStatus, '/' + ros_namespace + "/fmu/out/timesync_status"),
            (ApriltagMarker, '/' + ros_namespace + "/apriltag_info") 
        ]

        # ------- Messages ----------- 
        self.vehicle_status:VehicleStatus = None
        self.vehicle_attitude:VehicleAttitude = None
        self.vehicle_control_mode:VehicleControlMode = None
        self.vehicle_timesync_status:TimesyncStatus = None
        self.offboard_control_mode:OffboardControlMode = OffboardControlMode()

        self.offboard_control_mode.body_rate = False
        self.offboard_control_mode.attitude = False
        self.offboard_control_mode.position = True
        self.offboard_control_mode.velocity = False
        self.offboard_control_mode.acceleration = False

        # Create subscriptions
        for msg_class, topic in self._sub_topics:
            self._subscribers.append(
                self._attached_node.create_subscription(msg_class, topic, self._new_message_cb, self._qos) 
            )
        # Create publishers
        for msg_class, topic in self._pub_topics:
            self._publishers[msg_class] = self._attached_node.create_publisher(msg_class, topic, 10) 


    def _new_message_cb(self, msg):
        """ Callback for new messages """
        # self._attached_node.get_logger().info("Message received")
        if isinstance(msg, VehicleStatus):
            self.vehicle_status = copy.copy(msg)
        elif isinstance(msg, VehicleAttitude): 
            self.vehicle_attitude = copy.copy(msg)
        elif isinstance(msg, VehicleControlMode): 
            self.vehicle_control_mode = copy.copy(msg)
        elif isinstance(msg, TimesyncStatus): 
            self.vehicle_timesync_status = copy.copy(msg)
        elif isinstance(msg, ApriltagMarker): 
            self.camera.udpate_marker_pos(copy.copy(msg))
            d = self.camera.marker_distance
            self._attached_node.get_logger().info(f"Marker distance = {d}")
        else:
            self._attached_node.get_logger().warning(f"Message unknown: {msg}")

    def is_armed(self) -> bool:
        return self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def preflight_check(self) -> bool:
        return self.vehicle_status.pre_flight_checks_pass
    
    def system_id(self) -> int:
        return self.vehicle_status.system_id
    
    def component_id(self) -> int:
        return self.vehicle_status.component_id
    
    def nav_state(self) -> int:
        return self.vehicle_status.nav_state
    
    def get_now_timestamp(self) -> int:
        return int(self._attached_node.get_clock().now().nanoseconds / 1000)

    # ------- COMMANDS --------
    def send_arm_command(self) -> None:
        msg = self._create_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self._publishers.get(VehicleCommand).publish(msg)
    
    def send_disarm_command(self) -> None:
        msg = self._create_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self._publishers.get(VehicleCommand).publish(msg)

    def _create_vehicle_command(self, command, param1=0.0, param2=0.0) -> VehicleCommand:
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command  # command ID
        msg.target_system = self.system_id()  # system which should execute the command
        msg.target_component = self.component_id()  # component which should execute the command, 0 for all components
        msg.source_system = 10  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        return msg
    
    def offboard_flight_mode(self) -> None:
        command_msg = self._create_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self._publishers.get(VehicleCommand).publish(command_msg)

    def publish_offboard_control_signal(self) -> None:
        self.offboard_control_mode.timestamp = self.get_now_timestamp()
        self._publishers.get(OffboardControlMode).publish(self.offboard_control_mode)

    def publish_trajectory_setpoint(self, message:TrajectorySetpoint) -> None:
        self._publishers.get(TrajectorySetpoint).publish(message)
    
    def xrce_connected(self) -> bool:
        return self.vehicle_status and \
            self.vehicle_attitude and \
            self.vehicle_control_mode and \
            self.vehicle_timesync_status
    
    def get_marker_pos(self) -> tuple[float]:
        # TODO: da rivedere
        return self.camera.marker_3d_pos
        pass