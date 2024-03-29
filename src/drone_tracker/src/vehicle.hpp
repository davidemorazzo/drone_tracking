#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <chrono>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

enum MissionState {
	IDLE,
	PREFLIGHT_CHECK,
	MISSION,
	LANDING,
	TAKEOFF,
	POWEROFF
};

class Vehicle : public rclcpp::Node {

public:
	Vehicle();
	px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status = nullptr;
	px4_msgs::msg::VehicleControlMode::SharedPtr vehicle_control_mode = nullptr;
	px4_msgs::msg::TimesyncStatus::SharedPtr timesync_status = nullptr;
	px4_msgs::msg::OffboardControlMode::SharedPtr offboard_control_mode = nullptr;
	px4_msgs::msg::VehicleOdometry::SharedPtr vehicle_odometry = nullptr;
	float vehicle_starting_position[3] = {NAN, NAN, NAN};

	bool is_armed()			{return this->vehicle_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;};
	bool preflight_check()	{return this->vehicle_status->pre_flight_checks_pass;};
	int system_id()			{return this->vehicle_status->system_id;};
	int component_id()		{return this->vehicle_status->component_id;};
	int nav_state()			{return this->vehicle_status->nav_state;};
	bool xrce_connected();
	int get_now_timestamp() {return this->get_clock()->now().nanoseconds() / 1000;};

	void send_arm_command();														// Arm vehicle
	void send_disarm_command();														// Disamr vehicle
	void offboard_flight_mode();													// Change flight mode to offboard
	void publish_offboard_control_signal(); 										// Offboard control message as keepalive
	void publish_trajectory_setpoint(px4_msgs::msg::TrajectorySetpoint message);	// Set the position setpoint of the drone
	void mission_update();
	void publish_hor_acc_setpoint(float ax, float ay, float z, float yaw);			// Publish a setpoint to control horizontal acceleration
	void publish_pos_setpoint(float x, float y, float z, float yaw);				// Publish a position setpoint

	void broadcast_drone_tf(VehicleOdometry msg);
	void broadcast_camera_tf();

private:
	std::string ros_namespace;
	MissionState current_state, next_state;
	int mission_cb_cnt = 0;
	float acc_angle = 0;
	float flocking_ax = 0.0;
	float flocking_ay = 0.0;
	bool flocking_start = false;
	float land_pos_x = 0, land_pos_y = 0;

	rclcpp::QoS sub_qos = rclcpp::QoS(0)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

	rclcpp::QoS pub_qos = rclcpp::QoS(10)
		.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
		.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
		.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
	
	/*------ SUBSCRIBERS ----- */

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub = nullptr;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub = nullptr;
	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub = nullptr;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub = nullptr;	
	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr acceleration_sub = nullptr;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flocking_start_sub = nullptr;  

	void vehicle_status_cb(const VehicleStatus & message) {
		this->vehicle_status = std::make_shared<VehicleStatus>(std::move(message));};
	void vehicle_control_mode_cb(const VehicleControlMode & message) {
		this->vehicle_control_mode = std::make_shared<VehicleControlMode>(std::move(message));};
	void timesync_status_cb(const TimesyncStatus & message) {
		this->timesync_status = std::make_shared<TimesyncStatus>(std::move(message));};
	void flocking_start_cb(const std_msgs::msg::Bool & msg){
		this->flocking_start = msg.data;
	}
	void vehicle_odometry_cb(const VehicleOdometry & message);
	void estimator_cb(const std_msgs::msg::Float64MultiArray & message);

	/*------ PUBLISHERS ------ */

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
	
	/*-------- TIMERS ----------*/
	rclcpp::TimerBase::SharedPtr mission_timer = this->create_wall_timer(500ms, std::bind(& Vehicle::mission_update, this));
	rclcpp::TimerBase::SharedPtr offboard_timer = this->create_wall_timer(100ms, std::bind(& Vehicle::publish_offboard_control_signal, this));

	px4_msgs::msg::VehicleCommand create_vehicle_command(int command, float param1=NAN, float param2=NAN,
		float param3=NAN, float param4=NAN, float param5=NAN, float param6=NAN, float param7=NAN);
	
};



int main(int argc, char * argv[]){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Vehicle>());
	rclcpp::shutdown();
	return 0;
}

#endif
