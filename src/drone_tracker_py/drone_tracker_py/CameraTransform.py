from sensor_msgs.msg import Image

"""
Class to read the marker position from the sensor (in pixels) and transform it into
the 3 dimensional position.
"""

class CameraTransform:
	def __init__(self) -> None:
		self.marker_id : int = 0
		self.marker_detected : bool = False
		self.marker_3d_pos : list[float] = [0.0, 0.0, 0.0]

		self._marker_data = None
		self._marker_x : int = 0
		self._marker_y : int = 0
		self._marker_rot : float = 0
		self._marker_rect : tuple[int] = None

	def udpate_marker_pos(self, data : tuple) -> None:
		self._marker_data = data
		self.marker_id, self.marker_x, self.marker_y, self.marker_rot, self.marker_rect = self._marker_data
		# TODO: da scrivere il codice per la trasformazione
		
		