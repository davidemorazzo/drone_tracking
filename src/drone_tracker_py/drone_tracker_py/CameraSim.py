import rclpy
from rclpy.node import Node
from rclpy.qos import (
	QoSProfile, 
	QoSReliabilityPolicy, 
	QoSHistoryPolicy)
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from px4_msgs.msg import ApriltagMarker

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
		
		self._image_sub = self.create_subscription(Image, '/'+self.namespace+"/camera/raw_image", self.update_last_frame, qos_profile)
		self._marker_pub = self.create_publisher(ApriltagMarker, '/'+self.namespace+"/apriltag_info", qos_profile)

		self._last_frame : Image = None
		self._april_detected : bool = False
		self._x : int = 0
		self._y : int = 0
		self._rot : float = 0
		self._april_id :int = 0
		self._rect : tuple[int] = None

	def update_last_frame(self, image : Image) -> None:
		self._last_frame = image
		# TODO: COMPUTER VISION TO DETECT MARKER

		msg = ApriltagMarker()
		msg.marker_id = self._april_id
		msg.marker_x = self._x
		msg.marker_y = self._y
		msg._marker_rot = self._rot
		msg.marker_rect = self._rect
		self._marker_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    sub = CameraSim()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()