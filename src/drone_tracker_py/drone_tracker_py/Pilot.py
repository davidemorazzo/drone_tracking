from .Vehicle import *

IDLE = 1
PREFLIGHT_CHECK = 2
ARMING = 3
ARMED = 4
MISSION = 5
LANDING = 6
FAILSAFE = 7


class Pilot():
	def __init__(self) -> None:
		self.current_state = IDLE
		self.next_state = IDLE
		self.drone : Vehicle = None
	
	def set_vehicle(self, my_vehicle:Vehicle) -> None:
		self.drone = my_vehicle
		self.logger = self.drone._attached_node.get_logger()

	def mission_update(self) -> None:
		""" State machine function to execute the arming routine from
		 offboard mode.  """

		if not self.drone:
			return

		self.current_state = self.next_state

		if self.current_state == IDLE:
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
			pass				
		elif self.current_state == ARMED:
			pass				
		elif self.current_state == MISSION:
			pass				
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

	def _mission_func(self) -> None:
		pass