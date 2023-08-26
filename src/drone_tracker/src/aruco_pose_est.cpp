#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;


class ArucoPoseEst : public rclcpp::Node {

public:
	ArucoPoseEst() : Node("ArucoPoseEst"){
		if (std::string(this->get_namespace()) == "/"){
			this->ros_namespace = "";
		}else{
			this->ros_namespace = this->get_namespace();
		}

		marker_pos_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
			std::string(ros_namespace + "/camera/marker_pos"), 10, 
			std::bind(& ArucoPoseEst::sensor_cb, this, _1));

		rho_theta_pub = create_publisher<std_msgs::msg::Float64MultiArray>(
			std::string(ros_namespace + "/sensor_measure"), 10);

		camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
			std::string(ros_namespace + "/camera/camera_info"), 10, 
			std::bind(& ArucoPoseEst::camera_info_cb, this, _1));		

		this->tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    	this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	};

private:
	std::string ros_namespace;
	Eigen::Matrix3d cameraMatrix, cameraMatrixInv;
	Eigen::Vector<double, 5> distCoeffs;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rho_theta_pub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr marker_pos_sub;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
	
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	void sensor_cb(const std_msgs::msg::Float64MultiArray & msg){
		
		/* Markers pose estimation */
		if(! msg.data.empty()){

			/*Filter only the needed marker ids*/
			int marker_id = int(msg.data[0]);
			if (marker_id != 23) {return;}

			Eigen::Matrix4d camera_mtx = this->cameraTrasl(msg.data[1], msg.data[2]);

			this->broadcast_marker_tf(
				-camera_mtx(0,3),						
				-camera_mtx(1,3),					
				camera_mtx(2,3)						
			);

			/* Polar coordinates */
			try{
				std::vector<float> rho_theta = this->polar_coordinates(
					this->ros_namespace.substr(1), this->ros_namespace.substr(1)+"/marker");
				// RCLCPP_INFO(get_logger(), "Rho: %.3f Theta: %.3f", rho_theta[0], rho_theta[1]/3.15*180.0f);
				std_msgs::msg::Float64MultiArray rho_theta_msg;
				rho_theta_msg.data.push_back(float(rho_theta[0]));
				rho_theta_msg.data.push_back(float(rho_theta[1]));
				rho_theta_msg.data.push_back(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count() / 1E6);
				this->rho_theta_pub->publish(rho_theta_msg);
			}catch(...){
				RCLCPP_WARN(get_logger(), "Error generating polar coordinates");
			}
		}
		
	};

	void camera_info_cb(const sensor_msgs::msg::CameraInfo & msg){
		/* Read camera parameter topic and store the camera matrix and distorsion coefficients*/
		this->cameraMatrix << 	msg.k[0], msg.k[1], msg.k[2], 
								msg.k[3], msg.k[4], msg.k[5], 
								msg.k[6], msg.k[7], msg.k[8];
		this->cameraMatrixInv = cameraMatrix.inverse();
		this->distCoeffs = Eigen::Vector<double, 5>(msg.d.data());

		RCLCPP_INFO(get_logger(), "Camera matrix %f %f %f %f %f %f %f %f %f", cameraMatrix(0,0),
			cameraMatrix(0,1), cameraMatrix(0,2), cameraMatrix(1,0), cameraMatrix(1,1), 
			cameraMatrix(1,2), cameraMatrix(2,0),cameraMatrix(2,1), cameraMatrix(2,2));
		RCLCPP_INFO(get_logger(), "Distorsion  matrix %f %f %f %f %f", distCoeffs(0,0),
			distCoeffs(1,0), distCoeffs(2,0), distCoeffs(3,0), distCoeffs(4,0));

		/* Unsubscribe ?? */
		this->camera_info_sub = nullptr;
	}

	Eigen::Matrix4d cameraTrasl(double marker_cx, double marker_cy){
		/*Camera position*/
		geometry_msgs::msg::TransformStamped camera_tf;
		std::string camera_frame_id = this->ros_namespace.substr(1) + "/camera";
		camera_tf = tf_buffer_->lookupTransform(
			"map", camera_frame_id,
			tf2::TimePointZero);

		Eigen::Matrix<double, 3, 1> image_plane;

		float camera_height = camera_tf.transform.translation.z;
		image_plane(0, 0) = marker_cx * camera_height;
		image_plane(1, 0) = marker_cy * camera_height;
		image_plane(2, 0) = camera_height;
		
		Eigen::Matrix<double, 3, 1> coord = this->cameraMatrixInv * image_plane;

		Eigen::Matrix4d camera_matrix = Eigen::Matrix4d::Identity();
		camera_matrix(0,3) = -coord(1,0);
		camera_matrix(1,3) = -coord(0,0);
		camera_matrix(2,3) = -coord(2,0);

		return camera_matrix;
	}

	std::vector<float> polar_coordinates(std::string parent_id, std::string child_id){
		geometry_msgs::msg::TransformStamped t;
		t = tf_buffer_->lookupTransform(child_id, parent_id, tf2::TimePointZero);
		
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
	rclcpp::spin(std::make_shared<ArucoPoseEst>());
	rclcpp::shutdown();
	return 0;
}

