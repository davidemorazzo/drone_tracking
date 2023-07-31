#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/quaternion.hpp"
#include "opencv2/calib3d.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;


class Camera : public rclcpp::Node {

public:
	Camera() : Node("Camera"){
		if (std::string(this->get_namespace()) == "/"){
			this->ros_namespace = "";
		}else{
			this->ros_namespace = this->get_namespace();
		}

		serial_image_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
			std::string(ros_namespace + "/openmv_serial"), 10, 
			std::bind(& Camera::image_cb, this, _1));

		rho_theta_pub = create_publisher<std_msgs::msg::Float64MultiArray>(
			std::string(ros_namespace + "/sensor_measure"), 10);


		float c_matrix[3][3] = {
			{153.75337327, 0.0e+00, 77.20143553}, 
			{0.0e+00, 153.95236529, 58.43806698}, 
			{0.0e+00, 0.0e+00, 1.0e+00}
		};
		float dist_vect[5][1] = {{-0.40435636},{0.05816364},{0.00363366},{0.00228936},{0.1716887}};
		this->cameraMatrix = cv::Mat(3,3,CV_32F, c_matrix);
		this->cameraMatrixInv = cameraMatrix.inv();
		this->distCoeffs = cv::Mat(5,1, CV_32F, dist_vect);
		RCLCPP_INFO(get_logger(), "Camera matrix %f %f %f %f %f %f %f %f %f", cameraMatrix.at<float>(0,0),
			cameraMatrix.at<float>(0,1), cameraMatrix.at<float>(0,2), cameraMatrix.at<float>(1,0), cameraMatrix.at<float>(1,1), 
			cameraMatrix.at<float>(1,2), cameraMatrix.at<float>(2,0),cameraMatrix.at<float>(2,1), cameraMatrix.at<float>(2,2));
		RCLCPP_INFO(get_logger(), "Distorsion  matrix %f %f %f %f %f", distCoeffs.at<float>(0,0),
			distCoeffs.at<float>(1,0), distCoeffs.at<float>(2,0), distCoeffs.at<float>(3,0), distCoeffs.at<float>(4,0));

		this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    	this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	};

private:
	std::string ros_namespace;
	cv::Mat cameraMatrix, distCoeffs, cameraMatrixInv;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr serial_image_sub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rho_theta_pub;
	float vehicle_pos[3] = {0.0, 0.0, 0.0};
	float vehicle_q[4] = {0.0, 0.0, 0.0, 0.0};

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	
	void image_cb(const std_msgs::msg::Float64MultiArray &msg){
		/* Markers pose estimation */
		if(! msg.data.empty()){
			try{
				cv::Mat camera_mtx = this->cameraTrasl(msg.data);
				this->broadcast_marker_tf(
					camera_mtx.at<float>(0, 3),
					camera_mtx.at<float>(1, 3),
					-camera_mtx.at<float>(2, 3));

				/* Polar coordinates */
				std::vector<float> rho_theta = this->polar_coordinates(
					this->ros_namespace, 
					this->ros_namespace + "/marker");
				// RCLCPP_INFO(get_logger(), "Rho: %.3f Theta: %.3f", rho_theta[0], rho_theta[1]/3.15*180.0f);
				std_msgs::msg::Float64MultiArray rho_theta_msg;
				rho_theta_msg.data.push_back(float(rho_theta[0]));
				rho_theta_msg.data.push_back(float(rho_theta[1]));
				rho_theta_msg.data.push_back(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count() / 1E6);
				this->rho_theta_pub->publish(rho_theta_msg);
			
			}catch (...){}
		}
		
	};

	cv::Mat cameraTrasl(std::vector<double> info){
		cv::Mat out;
		if (int(info.size() >= 3)){
			cv::Mat image_plane = cv::Mat(3, 1, CV_32F);
			cv::Point2f center_point(info[1], info[2]);
			RCLCPP_INFO(get_logger(), "Point x=%.1f px y=%.1f px", center_point.x, center_point.y);

			/* Undistort point */
			// std::vector<cv::Point2d> points, undistorted_points;
			// points.push_back(center_point);
			// cv::undistortPoints(points, undistorted_points, this->cameraMatrix, this->distCoeffs);

			// RCLCPP_INFO(get_logger(), "Undistorted x=%.1f px y=%.1f px", undistorted_points[0].x, undistorted_points[0].y);
			
			geometry_msgs::msg::TransformStamped t;
			t = tf_buffer_->lookupTransform(
            	"map", this->ros_namespace,
            	tf2::TimePointZero);

			image_plane.at<float>(0,0) = center_point.x * t.transform.translation.z;
			image_plane.at<float>(1,0) = center_point.y * t.transform.translation.z;
			image_plane.at<float>(2,0) = t.transform.translation.z;

			// /*Test height*/
			// image_plane.at<float>(0,0) = center_point.x * 0.75;
			// image_plane.at<float>(1,0) = center_point.y * 0.75;
			// image_plane.at<float>(2,0) = 0.75;
			
			cv::Mat coord = cv::Mat(CV_32F,3,1); 
			coord = this->cameraMatrixInv * image_plane;

			cv::Matx<float, 4, 4> camera_matrix = cv::Matx<float, 4, 4>::eye();
			camera_matrix(0,3) = coord.at<float>(1,0);
			camera_matrix(1,3) = coord.at<float>(0,0);
			camera_matrix(2,3) = coord.at<float>(2,0);

			return cv::Mat(camera_matrix);
		}	
		return out;
	}

	std::vector<float> polar_coordinates(std::string parent_id, std::string child_id){
		geometry_msgs::msg::TransformStamped t;
			t = tf_buffer_->lookupTransform(
            	parent_id, child_id,
            	tf2::TimePointZero);
		
		float rho = sqrt(pow(t.transform.translation.x,2) + pow(t.transform.translation.y,2));
		float theta = atan2(t.transform.translation.y, t.transform.translation.x);
		
		std::vector<float> result;
		result.push_back(rho);
		result.push_back(theta);
		return result;
	}

	void broadcast_marker_tf(float x, float y, float z){
		geometry_msgs::msg::TransformStamped t;
		tf2::Quaternion q;
		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = this->ros_namespace + "/camera";
		t.child_frame_id = this->ros_namespace + "/marker";
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = z;
		t.transform.rotation.x = 0.0;
		t.transform.rotation.y = 0.0;
		t.transform.rotation.z = 0.0;
		t.transform.rotation.w = 1.0;
		this->tf_broadcaster->sendTransform(t);
	}
	
};



int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Camera>());
	rclcpp::shutdown();
	return 0;
}
