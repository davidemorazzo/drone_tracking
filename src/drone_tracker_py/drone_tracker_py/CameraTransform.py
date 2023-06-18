from sensor_msgs.msg import Image
from px4_msgs.msg import ApriltagMarker
from cv2 import aruco
import cv2
import numpy as np

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
		self.corners = None

		# Distance from object
		self._focal_lenght = 460 # Experimental value for gazebo
		self._marker_dimension = 0.23 #m
		self._k_matrix = np.matrix([[381.36246688113556, 0.0, 	320.5],
									[0.0, 381.36246688113556, 240.5],
									[0.0, 		0.0,   			1.0] ], dtype=float)
		self._d_matrix = np.zeros((1,5))
		

	def udpate_marker_pos(self, data : ApriltagMarker) -> None:
		self._marker_data = data
		# self.marker_distance = (self._marker_dimension * self._focal_lenght)  / self._marker_data.marker_rot
		self.corners = np.array([data.corner_1, data.corner_2, data.corner_3, data.corner_4])
		self.corners = np.ascontiguousarray(self.corners[:,:2]).reshape((4,1,2))
		# self.corners = [np.array([float(x[0]), float(x[1])]) for x in self.corners]
		rvec, tvec, markerPoints = cv2.solvePnP(np.array((4,1,cv2.CV_32FC3)), self.corners, self._k_matrix, self._d_matrix)
		
		# rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(self.corners, 0.23, 
		# 					     self._k_matrix, self._d_matrix)
		self.marker_3d_pos = tvec
		
		# TODO: da scrivere il codice per la trasformazione
		# self._focal_lenght = (self._marker_data.marker_rot * 1.1) / self._marker_dimension
		