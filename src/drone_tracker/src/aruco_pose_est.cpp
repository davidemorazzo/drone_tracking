#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include <random>
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

struct normal_random_variable{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}
    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean){
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }
    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;
    Eigen::VectorXd operator()() const{
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;
        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen);});
    }
};

class ArucoPoseEst : public rclcpp::Node {

public:
	ArucoPoseEst() : Node("ArucoPoseEst"){
		if (std::string(this->get_namespace()) == "/"){
			this->ros_namespace = "";
		}else{
			this->ros_namespace = this->get_namespace();
		}

		this->declare_parameter("fake_measure", false);
		bool fake_measure = this->get_parameter("fake_measure").get_parameter_value().get<bool>();

		/*Chooses if creating a timer to publish the position of a fake marker or to 
		listen to the camera measurements to identify a real marker.*/
		if (fake_measure){
			RCLCPP_INFO(get_logger(), "Publishing fake measurements...");
			this->fake_measure_timer = this->create_wall_timer(66ms, std::
				bind(& ArucoPoseEst::fake_measure_cb, this));
		}else{
			marker_pos_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
				std::string(ros_namespace + "/camera/marker_pos"), 10, 
				std::bind(& ArucoPoseEst::sensor_cb, this, _1));
		}

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
	rclcpp::TimerBase::SharedPtr fake_measure_timer;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	/* Create a transform between map to droneX/marker with fakes coordinates
	in order to create the bearing sensor readings and send it to the EASN
	estimator node.*/
	void fake_measure_cb(){
		double fake_marker_x = 0.0;
		double fake_marker_y = 0.0;
		// double fake_marker_z = 0.0;
		geometry_msgs::msg::TransformStamped drone_tf;
		try{
			drone_tf = tf_buffer_->lookupTransform("map", this->ros_namespace.substr(1),
			tf2::TimePointZero);
			
			double rho = sqrt(pow(drone_tf.transform.translation.x - fake_marker_x,2) + 
							pow(drone_tf.transform.translation.y - fake_marker_y,2));
			double theta = atan2(fake_marker_y - drone_tf.transform.translation.y, 
								fake_marker_x - drone_tf.transform.translation.x);
			/*Noise generation*/
			Eigen::MatrixXd covar(2,2);
			covar << 0.4*pow(cos(theta),2) + 0.1*pow(rho*sin(theta),2), (-0.4+0.1)*(cos(theta)*sin(theta)),
					(-0.4+0.1)*(cos(theta)*sin(theta)), 0.4*pow(sin(theta),2) + 0.1*pow(rho*cos(theta),2);
			normal_random_variable sample { covar };  
			std::vector<double> noise; 
			noise.resize(2);
			Eigen::Map<Eigen::VectorXd>(noise.data(), noise.size()) = sample();
			/*Send message*/
			std_msgs::msg::Float64MultiArray rho_theta_msg;
			rho_theta_msg.data.push_back(rho + noise[0]*0.3);
			rho_theta_msg.data.push_back(theta + noise[1]*0.3);
			rho_theta_msg.data.push_back(this->get_clock()->now().seconds());
			this->rho_theta_pub->publish(rho_theta_msg);

		}catch(std::exception &ex){
			RCLCPP_INFO(get_logger(), "%s", ex.what());
		}
	}

	void sensor_cb(const std_msgs::msg::Float64MultiArray & msg){
		
		/* Markers pose estimation */
		if(! msg.data.empty()){

			/*Filter only the needed marker ids*/
			// int marker_id = int(msg.data[0]);
			// if (marker_id != 23) {return;}

			try{
				Eigen::Matrix4d camera_mtx = this->cameraTrasl(msg.data[1], msg.data[2]);

				this->broadcast_marker_tf(
					-camera_mtx(0,3),						
					-camera_mtx(1,3),					
					camera_mtx(2,3)						
				);
			}catch(...){
				RCLCPP_WARN(get_logger(), "Error calculating marker position");
			}

			try{
			/* Polar coordinates */
				std::vector<float> rho_theta = this->polar_coordinates(
					this->ros_namespace.substr(1), this->ros_namespace.substr(1)+"/marker");

				std_msgs::msg::Float64MultiArray rho_theta_msg;
				rho_theta_msg.data.push_back(rho_theta[0]);
				rho_theta_msg.data.push_back(rho_theta[1]);
				rho_theta_msg.data.push_back(this->get_clock()->now().seconds());
				this->rho_theta_pub->publish(rho_theta_msg);
			}catch(tf2::TransformException &ex){
				RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
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
		rclcpp::Time now = this->get_clock()->now();
		std::string camera_frame_id = this->ros_namespace.substr(1) + "/camera";
		camera_tf = tf_buffer_->lookupTransform(
			"map", camera_frame_id,
			now, 200ms);

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
		geometry_msgs::msg::TransformStamped drone_tf, marker_tf;
		drone_tf = tf_buffer_->lookupTransform("map", parent_id, tf2::TimePointZero);
		marker_tf = tf_buffer_->lookupTransform("map", child_id, tf2::TimePointZero);
		double delta_x = marker_tf.transform.translation.x - drone_tf.transform.translation.x;
		double delta_y = marker_tf.transform.translation.y - drone_tf.transform.translation.y;
		float rho = sqrt(pow(delta_x,2) + pow(delta_y,2));
		float theta = atan2(delta_y, delta_x);
		
		// float rho = sqrt(pow(t.transform.translation.x,2) + pow(t.transform.translation.y,2));
		// float theta = atan2(t.transform.translation.y, t.transform.translation.x);
		
		std::vector<float> result;
		result.push_back(rho);
		result.push_back(theta);
		return result;
	}

	void broadcast_marker_tf(float x, float y, float z){
		geometry_msgs::msg::TransformStamped t;
		tf2::Quaternion q;
		t.header.stamp = this->get_clock()->now();
		t.header.frame_id = this->ros_namespace.substr(1) + "/camera";
		t.child_frame_id = this->ros_namespace.substr(1) + "/marker";
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

