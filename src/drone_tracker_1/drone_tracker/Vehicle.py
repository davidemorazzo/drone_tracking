import rclpy
from rclpy.clock import Clock

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import TimesyncStatus
from px4_msgs.msg import VehicleCommand


class Vehicle():
# Class representing the vehicle. Subscribe to PX4 topics to update its state

    def __init__(self, node:rclpy.Node, ros_namespace:str="") -> None:

        self._subscribers = []
        self._publishers = {}
        self._attached_node = node

        self._pub_topics:list = [
            (VehicleCommand, ros_namespace + "/fmu/in/vehicle_command")
        ]

        self._sub_topics:list = [
            (VehicleStatus, ros_namespace + "/fmu/out/vehicle_status"),
            (VehicleAttitude, ros_namespace + "/fmu/out/Vehicle_attitude"),
            (VehicleControlMode, ros_namespace + "/fmu/out/vehicle_control_mode"),
            (TimesyncStatus, ros_namespace + "/fmu/out/timesync_status")
        ]

        # ------- Messages ----------- 
        self.vehicle_status:VehicleStatus = None
        self.vehicle_odometry:VehicleOdometry = None
        self.vehicle_attitude:VehicleAttitude = None
        self.vehicle_control_mode:VehicleControlMode = None
        self.vehicle_timesync_status:TimesyncStatus = None

        # Create subscriptions
        for msg_class, topic in self._sub_topics:
            self._subscribers.append(
                self._attached_node.create_subscription(msg_class, topic, self._new_message_cb, 10) 
            )
        # Create publishers
        for msg_class, topic in self._pub_topics:
            self._publishers[msg_class] = self._attached_node.create_publisher(msg_class, topic, 10) 


    def _new_message_cb(self, msg):
        """ Callback for new messages """

        if isinstance(msg, VehicleStatus):
            self.vehicle_status = msg
        elif isinstance(msg, VehicleOdometry): 
            self.vehicle_odometry = msg
        elif isinstance(msg, VehicleAttitude): 
            self.vehicle_attitude = msg
        elif isinstance(msg, VehicleControlMode): 
            self.vehicle_control_mode = msg
        elif isinstance(msg, TimesyncStatus): 
            self.vehicle_timesync_status = msg

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


