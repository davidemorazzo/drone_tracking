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

		double c_matrix[9] = {381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0};
		cameraMatrix = cv::Mat(3,3,CV_64FC1, &c_matrix);
		double dist_vect[5] = {0,0,0,0,0};
		distCoeffs = cv::Mat(5,1, CV_64FC1, &dist_vect);
		RCLCPP_INFO(get_logger(), "Camera matrix %f %f %f %f %f %f %f %f %f", cameraMatrix.at<double>(0,0),
			cameraMatrix.at<double>(0,1), cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,0), cameraMatrix.at<double>(1,1), 
			cameraMatrix.at<double>(1,2), cameraMatrix.at<double>(2,0),cameraMatrix.at<double>(2,1), cameraMatrix.at<double>(2,2));
		RCLCPP_INFO(get_logger(), "Distorsion  matrix %f %f %f %f %f", distCoeffs.at<double>(0,0),
			distCoeffs.at<double>(1,0), distCoeffs.at<double>(2,0), distCoeffs.at<double>(3,0), distCoeffs.at<double>(4,0));
	};

private:
	cv_bridge::CvImagePtr cv_ptr;
	std::string ros_namespace;
	sensor_msgs::msg::Image::SharedPtr last_image = nullptr;
	float markerLength = 230; // [m]
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
		cv_ptr = cv_bridge::toCvCopy(msg, std::string("bgra8"));

		/* OPENCV stuff */
		cv::Mat inputImage;
		cv_ptr->image.copyTo(inputImage);
		cv::cvtColor(cv_ptr->image, inputImage, cv::COLOR_BGR2GRAY);
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
		const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

		// cv::Mat output_img = inputImage.clone();
		// cv::aruco::drawDetectedMarkers(output_img, markerCorners, markerIds);
		// cv::imwrite("camera.png", output_img);
		
		/* Markers pose estimation */
		std::vector<cv::Vec3d> rvecs, tvecs;
		
		// Set coordinate system
		cv::Mat objPoints(4, 1, CV_32FC3);
		objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
		objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
		
		// cv::aruco::estimatePoseSingleMarkers(markerCorners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
		
		for (int i = 0; i < int(markerIds.size()); i++) {
			rvecs.clear();
			tvecs.clear();
			cv::Vec3f rvec, tvec;
 			solvePnP(objPoints, markerCorners.at(i), cameraMatrix, distCoeffs, rvec, tvec);
			rvecs.push_back(rvec);
			tvecs.push_back(tvec);
		}
		
		for (int i =0; i < int(markerIds.size()); i++){
			RCLCPP_INFO(get_logger(), "Translation vec: %f %f %f", tvecs[i].val[0], tvecs[i].val[1],
				tvecs[i].val[2]);
		}

		cv::Mat outputImage;
		inputImage.copyTo(outputImage);
		cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
		for (int i = 0; i < rvecs.size(); ++i) {
			auto rvec = rvecs[i];
			auto tvec = tvecs[i];
			cv::drawFrameAxes(outputImage, cameraMatrix, distCoeffs, rvec, tvec, 1);
		}
		cv::imwrite("camera.png", outputImage);
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

