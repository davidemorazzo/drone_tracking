#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace px4_msgs::msg;

class Vehicle {

public:
	Vehicle(std::string ros_namespace, rclcpp::Node::SharedPtr attached_node);
	px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status;
	px4_msgs::msg::VehicleAttitude::SharedPtr vehicle_attitude;
	px4_msgs::msg::VehicleControlMode::SharedPtr vehicle_control_mode;
	px4_msgs::msg::TimesyncStatus::SharedPtr timesync_status;
	px4_msgs::msg::OffboardControlMode::SharedPtr offboard_control_mode;

	bool is_armed()			{return this->vehicle_status->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;};
	bool preflight_check()	{return this->vehicle_status->pre_flight_checks_pass;};
	int system_id()			{return this->vehicle_status->system_id;};
	int component_id()		{return this->vehicle_status->component_id;};
	int nav_state()			{return this->vehicle_status->nav_state;};
	int get_now_timestamp() {return this->attached_node->get_clock()->now().nanoseconds() / 1000;};

	void send_arm_command();														// Arm vehicle
	void send_disarm_command();														// Disamr vehicle
	void offboard_flight_mode();													// Change flight mode to offboard
	void publish_offboard_control_signal(); 										// Offboard control message as keepalive
	void publish_trajectory_setpoint(px4_msgs::msg::TrajectorySetpoint message);	// Set the position setpoint of the drone
	rclcpp::Node::SharedPtr attached_node = nullptr;

private:
	std::string ros_namespace;
	/*------ SUBSCRIBERS ----- */

	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub;
	rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub;		  

	void vehicle_status_cb(const VehicleStatus & message) {
		this->vehicle_status = std::make_shared<VehicleStatus>(std::move(message));};
	void vehicle_attitude_cb(const VehicleAttitude & message) {
		this->vehicle_attitude = std::make_shared<VehicleAttitude>(std::move(message));};
	void vehicle_control_mode_cb(const VehicleControlMode & message) {
		this->vehicle_control_mode = std::make_shared<VehicleControlMode>(std::move(message));};
	void timesync_status_cb(const TimesyncStatus & message) {
		this->timesync_status = std::make_shared<TimesyncStatus>(std::move(message));};

	/*------ PUBLISHERS ------ */

	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub;
	
	px4_msgs::msg::VehicleCommand create_vehicle_command(int command, float param1=0.0, float param2=0.0);
	
};

#endif
