#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

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

		odom_sub = create_subscription<px4_msgs::msg::VehicleOdometry>(
			std::string(ros_namespace + "/fmu/in/vehicle_visual_odometry"), 10, 
			std::bind(& Camera::odometry_cb, this, _1));

		marker_pose_pub = create_publisher<geometry_msgs::msg::PointStamped>(
			std::string(ros_namespace + "/marker_pose"), 10);

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
	};

private:
	std::string ros_namespace;
	sensor_msgs::msg::Image::SharedPtr last_image = nullptr;
	float markerLength = 0.23; // [mm]
	cv::Mat cameraMatrix, distCoeffs, cameraMatrixInv;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub;
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr serial_image_sub;
	rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr marker_pose_pub;
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rho_theta_pub;
	float vehicle_pos[3] = {0.0, 0.0, 0.0};
	float vehicle_q[4] = {0.0, 0.0, 0.0, 0.0};
	std::string dev_name = "/dev/ttyUSB0";
	 


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
		this->vehicle_q[0] = msg.q[0];
		this->vehicle_q[1] = msg.q[1];
		this->vehicle_q[2] = msg.q[2];
		this->vehicle_q[3] = msg.q[3];
	}
	
	void image_cb(const std_msgs::msg::Float64MultiArray &msg){
		/* Markers pose estimation */
		if(! msg.data.empty()){

			cv::Mat camera_mtx = this->cameraTrasl(msg.data);
			cv::Mat drone_mtx = this->drone_matrix();
			cv::Mat marker_pose = camera_mtx * drone_mtx;
			// std::string out = "";
			// out << camera_mtx;
			// RCLCPP_INFO(get_logger(), out.c_str());

			geometry_msgs::msg::PointStamped point;
			point.header.frame_id = "map";
			point.header.stamp = this->now();
			point.point.x = marker_pose.at<float>(0,3);
			point.point.y = marker_pose.at<float>(1,3);
			point.point.z = marker_pose.at<float>(2,3);
			this->marker_pose_pub->publish(point);

			/* Polar coordinates */
			std::vector<float> rho_theta = this->polar_coordinates(marker_pose, drone_mtx);
			// RCLCPP_INFO(get_logger(), "Rho: %.3f Theta: %.3f", rho_theta[0], rho_theta[1]/3.15*180.0f);
			std_msgs::msg::Float64MultiArray rho_theta_msg;
			rho_theta_msg.data.push_back(float(rho_theta[0]));
			rho_theta_msg.data.push_back(float(rho_theta[1]));
			rho_theta_msg.data.push_back(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count() / 1E6);
			this->rho_theta_pub->publish(rho_theta_msg);
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
			
			image_plane.at<float>(0,0) = center_point.x * -this->vehicle_pos[2];
			image_plane.at<float>(1,0) = center_point.y * -this->vehicle_pos[2];
			image_plane.at<float>(2,0) = -this->vehicle_pos[2];

			// /*Test height*/
			// image_plane.at<float>(0,0) = center_point.x * 0.75;
			// image_plane.at<float>(1,0) = center_point.y * 0.75;
			// image_plane.at<float>(2,0) = 0.75;
			
			cv::Mat coord = cv::Mat(CV_32F,3,1); 
			coord = this->cameraMatrixInv * image_plane;

			cv::Matx<float, 4, 4> camera_matrix = cv::Matx<float, 4, 4>::eye();
			camera_matrix(0,3) = -coord.at<float>(1,0);
			camera_matrix(1,3) = -coord.at<float>(0,0);
			camera_matrix(2,3) = -coord.at<float>(2,0);

			return cv::Mat(camera_matrix);
		}	
		return out;
	}

	cv::Mat drone_matrix(){
		// cv::Mat drone_matrix = cv::Mat::eye(4,4,CV_32F);
		cv::Quat<float> drone_quaternion = cv::Quat<float>(vehicle_q[0],vehicle_q[1],vehicle_q[2],vehicle_q[3]);
		cv::Matx<float, 4, 4> drone_matrix = drone_quaternion.toRotMat4x4();
		drone_matrix(0,3) =  this->vehicle_pos[0]; // add 15cm for camera position
		drone_matrix(1,3) = -this->vehicle_pos[1];
		drone_matrix(2,3) = -this->vehicle_pos[2];
		return cv::Mat(drone_matrix);
	}

	std::vector<float> polar_coordinates(cv::Mat marker_mtx, cv::Mat drone_mtx){
		float delta_x = marker_mtx.at<float>(0,3) - drone_mtx.at<float>(0,3);
		float delta_y = marker_mtx.at<float>(1,3) - drone_mtx.at<float>(1,3);
		float rho = sqrt(pow(delta_x,2) + pow(delta_y,2));
		float theta = atan2(delta_y, delta_x);
		std::vector<float> result;
		result.push_back(rho);
		result.push_back(theta);
		return result;
	}
	
};



int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Camera>());
	rclcpp::shutdown();
	return 0;
}

