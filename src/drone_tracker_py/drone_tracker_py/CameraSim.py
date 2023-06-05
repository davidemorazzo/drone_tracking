import rclpy
from rclpy.node import Node
from rclpy.qos import (
	QoSProfile, 
	QoSReliabilityPolicy, 
	QoSHistoryPolicy)
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from px4_msgs.msg import ApriltagMarker

import apriltag
import cv2
from cv_bridge import CvBridge
import math

"""Camera Computer vision simulator from Gazebo camera"""

class CameraSim(Node) :
	def __init__(self) -> None:
		super().__init__('camerasim_node')
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1
		)
		self.declare_parameter("namespace", "drone1")
		self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
		self.bridge = CvBridge()
		
		self._image_sub = self.create_subscription(Image, '/'+self.namespace+"/camera/image_raw", self.update_last_frame, qos_profile)
		self._marker_pub = self.create_publisher(ApriltagMarker, '/'+self.namespace+"/apriltag_info", qos_profile)
		self._detect_timer = self.create_timer(0.1, self._detect_marker)

		self._last_frame : Image = None
		self._april_detected : bool = False
		self._x : int = 0
		self._y : int = 0
		self._rot : float = 0.0
		self._april_id :int = 0
		self._rect : tuple[int] = (0,0,0,0)

	def update_last_frame(self, image : Image) -> None:
		self._last_frame = image
	
	def distance(self, pta, ptb) -> float:
		return math.sqrt((pta[0]-ptb[1])**2 + (pta[1]-ptb[1])**2)

	def _detect_marker(self):
		image = self.bridge.imgmsg_to_cv2(self._last_frame, 'bgra8')
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# gray = cv2.getRectSubPix(gray, [320, 240], [320, 240])
		options = apriltag.DetectorOptions(families="tag36h11")
		detector = apriltag.Detector(options)
		results = detector.detect(gray)

		# Multiple results can be found
		for r in results:
			# extract the bounding box (x, y)-coordinates for the AprilTag
			# and convert each of the (x, y)-coordinate pairs to integers
			(ptA, ptB, ptC, ptD) = r.corners
			ptB = (int(ptB[0]), int(ptB[1]))
			ptC = (int(ptC[0]), int(ptC[1]))
			ptD = (int(ptD[0]), int(ptD[1]))
			ptA = (int(ptA[0]), int(ptA[1]))
			(cX, cY) = (int(r.center[0]), int(r.center[1]))

			max_diag = max(
				self.distance(ptA, ptB),
				self.distance(ptA, ptC),
				self.distance(ptA, ptD),
				self.distance(ptB, ptC),
				self.distance(ptB, ptD),
				self.distance(ptC, ptD),			
			)
		
			msg = ApriltagMarker()
			msg.marker_id = r.tag_id
			msg.marker_x = cX
			msg.marker_y = cY
			msg._marker_rot = max_diag / math.sqrt(2)
			msg.marker_rect = self._rect
			self._marker_pub.publish(msg)
		
		# cv2.imwrite("/home/judocero/Scrivania/drone_tracking/gray.png", gray)



def main(args=None):
    rclpy.init(args=args)
    sub = CameraSim()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()