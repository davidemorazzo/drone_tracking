#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
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

		float c_matrix[9] = {381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0};
		cameraMatrix = cv::Mat(3,3,CV_32F, c_matrix);
		float dist_vect[5] = {0,0,0,0,0};
		distCoeffs = cv::Mat(1,5, CV_32F, dist_vect);

	};

private:
	cv_bridge::CvImagePtr cv_ptr;
	std::string ros_namespace;
	sensor_msgs::msg::Image::SharedPtr last_image = nullptr;
	float markerLength = 0.23; // [m]
	cv::Mat cameraMatrix, distCoeffs;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;


	rclcpp::QoS sub_qos = rclcpp::QoS(0)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

	rclcpp::QoS pub_qos = rclcpp::QoS(10)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
	
	void image_cb(const sensor_msgs::msg::Image &msg){
		// last_image = std::make_shared<sensor_msgs::msg::Image>(std::move(msg));
		//  cv_ptr = cv_bridge::toCvCopy(msg,'BGR8' );
		cv_ptr = cv_bridge::toCvCopy(msg, std::string("bgra8"));

		/* OPENCV stuff */
		cv::Mat inputImage;
		cv::cvtColor(cv_ptr->image, inputImage, cv::COLOR_BGR2GRAY);
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
		cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		// detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);
		cv::aruco::detectMarkers(inputImage, &dictionary, markerCorners, markerIds, &detectorParams,
				rejectedCandidates);
		
		RCLCPP_INFO(get_logger(),"segmentation test");

		/* Markers pose estimation */
		cv::Mat cameraMatrix, distCoeffs;
		std::vector<cv::Vec3d> rvecs, tvecs;
		// You can read camera parameters from tutorial_camera_params.yml
		// readCameraParameters(cameraParamsFilename, cameraMatrix, distCoeffs); // This function is implemented in aruco_samples_utility.hpp
		
		// Set coordinate system
		cv::Mat objPoints(4, 1, CV_32FC3);
		objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
		
		// Calculate pose for each marker
		// Resulting translation is in tvecs[i] for each detected marker
		for (int i = 0; i < int(markerCorners.size()); i++) {
			solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
			// RCLCPP_INFO(get_logger(), "Translation vec: %.2f %.2f %.2f", tvecs.at(i)[0],tvecs.at(i)[1],
			// 	tvecs.at(i)[2]);
			 RCLCPP_INFO(get_logger(),"solvePnP");
		}
	};


	/*------ PUBLISHERS ------ */

	
	/*-------- TIMERS ----------*/
	
};



int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Camera>());
	rclcpp::shutdown();
	return 0;
}

