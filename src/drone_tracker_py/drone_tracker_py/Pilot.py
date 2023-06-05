from .Vehicle import *
from rclpy.node import Node

IDLE = 1
PREFLIGHT_CHECK = 2
ARMING = 3
MISSION = 5
LANDING = 6
FAILSAFE = 7


class Pilot(Node):
	def __init__(self) -> None:
		super().__init__('DronePilot')
		# ------- PARAMETERS ------- #
		self.declare_parameter("namespace", "drone1")
		self.declare_parameter("simulation", True)
		self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
		self.simulation = self.get_parameter("simulation").get_parameter_value().bool_value
		# --------- TIMERS -------- #
		self.offboard_timer = self.create_timer(0.1, self.offboard_callback)
		self.mission_timer = self.create_timer(0.5, self.mission_update)
		
		self.current_state = IDLE
		self.next_state = IDLE
		self.drone : Vehicle = Vehicle(self, self.namespace)
		self.logger = self.get_logger()
	
	def mission_update(self) -> None:
		""" State machine function to execute the arming routine from
		 offboard mode.  """

		if not self.drone:
			return

		self.current_state = self.next_state

		# if self.drone.vehicle_status.failsafe:
		# 	self.logger.warning(f"Vehicle {self.drone.namespace} in failsafe mode")
		# 	self.current_state = FAILSAFE

		if self.current_state == IDLE:
			if not self.drone.xrce_connected():
				self.next_state = IDLE
				return
			
			if self.drone.nav_state() == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
				self.logger.info("IDLE=>PREFLIGHT_CHECK")
				self.next_state = PREFLIGHT_CHECK
			else:
				self.drone.offboard_flight_mode()
				self.next_state = IDLE

		elif self.current_state == PREFLIGHT_CHECK:
			if self.drone.preflight_check():
				self.next_state = ARMING
				self.logger.info("PREFLIGHT_CHECK=>ARMING")
			else:
				self.next_state = PREFLIGHT_CHECK

		elif self.current_state == ARMING:
			if self.drone.is_armed():
				self.next_state = MISSION
				self.logger.info("ARMING=>MISSION")
			else:
				self.drone.send_arm_command()

		elif self.current_state == MISSION:
			if self._mission_func():
				self.next_state = LANDING
				self.logger.info("MISSION=>LANDING")
			else:
				self.next_state = MISSION

		elif self.current_state == LANDING:
			pass
		elif self.current_state == FAILSAFE:
			pass		
		else:
			self.logger.warning("Unknown mission state. Fallback to IDLE state")
			self.next_state = IDLE

		

	def offboard_callback(self) -> None:
		if not self.drone:
			return
		self.drone.publish_offboard_control_signal()

	def _mission_func(self) -> bool:
		msg = TrajectorySetpoint()
		msg.timestamp = self.drone.get_now_timestamp()
		if self.drone.namespace == "drone2":
			msg.position = [-0.2, 0.0, -1.0]
			msg.yaw = 0.0
		elif self.drone.namespace == "drone3":
			msg.position = [5.0, 0.0, -1.0]
			msg.yaw = 3.1415

		self.drone.publish_trajectory_setpoint(msg)
		return False
	
def main(args=None):
    rclpy.init(args=args)
    print("Starting Pilot node...\n")
    offboard_control = Pilot()
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()