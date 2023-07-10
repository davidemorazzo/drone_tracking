#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "opencv2/opencv.hpp"
// #include "opencv2/aruco.hpp"
#include "cv_bridge/cv_bridge.h"


using namespace std::chrono_literals;
using std::placeholders::_1;


class Camera : public rclcpp::Node {

public:
	Camera() : Node("Camera"){
		if (std::string(this->get_namespace()) == "/"){
			this->ros_namespace = "";
		}else{
			this->ros_namespace = this->get_namespace();
		}

		image_sub = create_subscription<sensor_msgs::msg::Image>(
			std::string(ros_namespace + "/camera/image_raw"), 10, 
			std::bind(& Camera::image_cb, this, _1));
		
		odom_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
			std::string(ros_namespace + "/fmu/in/vehicle_visual_odometry"), 10, 
			std::bind(& Camera::odometry_cb, this, _1));

		marker_pose_pub = create_publisher<geometry_msgs::msg::PointStamped>(
			std::string(ros_namespace + "/marker_pose"), 10);

		float c_matrix[3][3] = {
			{381.3624668, 0.0e+00, 320.5}, 
			{0.0e+00, 381.3624668, 240.5}, 
			{0.0e+00, 0.0e+00, 1.0e+00}
		};
		float dist_vect[5][1] = {{0},{0},{0},{0},{0}};
		this->cameraMatrix = cv::Mat(3,3,CV_32F, c_matrix);
		this->cameraMatrixInv = cameraMatrix.inv();
		this->distCoeffs = cv::Mat(5,1, CV_32F, dist_vect);
		RCLCPP_INFO(get_logger(), "Camera matrix %f %f %f %f %f %f %f %f %f", cameraMatrix.at<float>(0,0),
			cameraMatrix.at<float>(0,1), cameraMatrix.at<float>(0,2), cameraMatrix.at<float>(1,0), cameraMatrix.at<float>(1,1), 
			cameraMatrix.at<float>(1,2), cameraMatrix.at<float>(2,0),cameraMatrix.at<float>(2,1), cameraMatrix.at<float>(2,2));
		RCLCPP_INFO(get_logger(), "Distorsion  matrix %f %f %f %f %f", distCoeffs.at<double>(0,0),
			distCoeffs.at<double>(1,0), distCoeffs.at<double>(2,0), distCoeffs.at<double>(3,0), distCoeffs.at<double>(4,0));
	};

private:
	cv_bridge::CvImagePtr cv_ptr;
	std::string ros_namespace;
	sensor_msgs::msg::Image::SharedPtr last_image = nullptr;
	float markerLength = 0.23; // [mm]
	cv::Mat cameraMatrix, distCoeffs, cameraMatrixInv;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr marker_pose_pub;
	float vehicle_pos[3] = {0.0, 0.0, 0.0};
	 


	rclcpp::QoS sub_qos = rclcpp::QoS(0)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

	rclcpp::QoS pub_qos = rclcpp::QoS(10)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

	void odometry_cb(const px4_msgs::msg::VehicleOdometry &msg){
		this->vehicle_pos[0] = msg.position[0];
		this->vehicle_pos[1] = msg.position[1];
		this->vehicle_pos[2] = msg.position[2];
	}
	
	void image_cb(const sensor_msgs::msg::Image &msg){
		cv_ptr = cv_bridge::toCvCopy(msg, std::string("bgra8"));

		/* OPENCV stuff */
		cv::Mat inputImage;
		cv_ptr->image.copyTo(inputImage);
		cv::cvtColor(cv_ptr->image, inputImage, cv::COLOR_BGR2GRAY);

		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
		cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::aruco::ArucoDetector detector(dictionary, detectorParams);
		detector.detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);
		
		/* Markers pose estimation */
		if(! markerCorners.empty()){
			cv::Mat coord = this->cameraTrasl(markerCorners.at(0));
			std::string out = "Traslation: ";
			out << coord;
			RCLCPP_INFO(get_logger(), out.c_str());

			geometry_msgs::msg::PointStamped point;
			point.header.frame_id = "map";
			point.header.stamp = this->now();
			point.point.x = coord.at<float>(0,0);
			point.point.y = coord.at<float>(0,1);
			point.point.z = coord.at<float>(0,2);
			this->marker_pose_pub->publish(point);

		}
		
	};

	cv::Mat cameraTrasl(std::vector<cv::Point2f> points){
		cv::Mat out;
		if (int(points.size() == 4)){
			cv::Mat image_plane = cv::Mat(3, 1, CV_32F);
			cv::Point2f center_point;
			center_point = points[0] + points[2];
			center_point = center_point / 2;
			
			image_plane.at<float>(0,0) = center_point.x / -this->vehicle_pos[2];
			image_plane.at<float>(1,0) = center_point.y / -this->vehicle_pos[2];
			image_plane.at<float>(2,0) = -this->vehicle_pos[2];
			
			cv::Mat coord = cv::Mat(CV_32F,3,1); 
			coord = this->cameraMatrixInv * image_plane;
			return coord;
		}	
		return out;
	}


	/*------ PUBLISHERS ------ */

	
	/*-------- TIMERS ----------*/
	
};



int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Camera>());
	rclcpp::shutdown();
	return 0;
}

