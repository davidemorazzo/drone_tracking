from sensor_msgs.msg import Image
from px4_msgs.msg import ApriltagMarker

"""
Class to read the marker position from the sensor (in pixels) and transform it into
the 3 dimensional position.
"""

class CameraTransform:
	def __init__(self) -> None:
		self.marker_id : int = 0
		self.marker_detected : bool = False

		self._marker_data = None
		self.marker_3d_pos : list[float] = [0.0, 0.0, 0.0]
		self._marker_direction = [0.0, 0.0]

		# Distance from object
		self._focal_lenght = 460 # Experimental value for gazebo
		self._marker_dimension = 0.23 #m
		

	def udpate_marker_pos(self, data : ApriltagMarker) -> None:
		self._marker_data = data
		self.marker_distance = (self._marker_dimension * self._focal_lenght)  / self._marker_data.marker_rot
		
		# TODO: da scrivere il codice per la trasformazione
		# self._focal_lenght = (self._marker_data.marker_rot * 1.1) / self._marker_dimension
		