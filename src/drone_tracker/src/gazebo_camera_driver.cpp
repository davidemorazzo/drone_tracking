#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/quaternion.hpp"
#include "cv_bridge/cv_bridge.h"


using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;


class GazeboCamDriver : public rclcpp::Node {

public:
	GazeboCamDriver() : Node("GazeboCamDriver"){
		if (std::string(this->get_namespace()) == "/"){
			this->ros_namespace = "";
		}else{
			this->ros_namespace = this->get_namespace();
		}

		image_sub = create_subscription<sensor_msgs::msg::Image>(
			std::string(ros_namespace + "/camera/image_raw"), 10, 
			std::bind(& GazeboCamDriver::image_cb, this, _1));
		
		marker_pos_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(
			std::string(ros_namespace + "/camera/marker_pos"), 10);


		this->tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
		this->broadcast_camera_tf();
	};

private:
	cv_bridge::CvImagePtr cv_ptr;
	std::string ros_namespace;
	sensor_msgs::msg::Image::SharedPtr last_image = nullptr;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr marker_pos_pub;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
	std::vector<int> markerIds;
	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
	cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::ArucoDetector detector = cv::aruco::ArucoDetector(dictionary, detectorParams);

	
	void image_cb(const sensor_msgs::msg::Image &msg){
		cv_ptr = cv_bridge::toCvCopy(msg, std::string("bgra8"));

		/* OPENCV stuff */
		cv::Mat inputImage;
		cv_ptr->image.copyTo(inputImage);
		cv::cvtColor(cv_ptr->image, inputImage, cv::COLOR_BGR2GRAY);

		/* Locate markers with OpenCV */
		markerIds.clear();
		markerCorners.clear();
		rejectedCandidates.clear();
		detector.detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);
		
		/* Send marker positions */
		for(int i=0; i<(int)markerCorners.size(); i++){
			std::vector<cv::Point2f> points = markerCorners.at(i);
			cv::Point2f center_point;
			center_point = points[0] + points[2];
			center_point = center_point / 2;
			int marker_id = markerIds.at(i);

			std_msgs::msg::Float64MultiArray msg;
			msg.data.clear();
			msg.data.push_back(double(marker_id));
			msg.data.push_back(center_point.x);
			msg.data.push_back(center_point.y);
			
			this->marker_pos_pub->publish(msg);
		}
	}

	void broadcast_camera_tf(){
		geometry_msgs::msg::TransformStamped t;
		tf2::Quaternion q;
		q.setEuler(3.1415, 3.1415, 0.0);
		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = this->ros_namespace;
		t.child_frame_id = this->ros_namespace + "/camera";
		t.transform.translation.x = 0.15;
		t.transform.translation.y = 0.0;
		t.transform.translation.z = 0.0;
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();
		this->tf_static_broadcaster_->sendTransform(t);
	}
	
};



int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GazeboCamDriver>());
	rclcpp::shutdown();
	return 0;
}

