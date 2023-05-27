#include "vehicle.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace px4_msgs::msg;
using std::placeholders::_1;

Vehicle::Vehicle(std::string ros_namespace, rclcpp::Node::SharedPtr attached_node){
	this->ros_namespace = ros_namespace;
	this->attached_node = attached_node;
	
	/*Offboard control mode message*/
	this->offboard_control_mode = OffboardControlMode::SharedPtr();
	this->offboard_control_mode->timestamp = this->get_now_timestamp();
	this->offboard_control_mode->body_rate = false;
	this->offboard_control_mode->attitude = false;
	this->offboard_control_mode->position = true;
	this->offboard_control_mode->velocity = false;
	this->offboard_control_mode->acceleration = false;

	/*Creating publishers*/
	this->vehicle_command_pub = this->attached_node->create_publisher<VehicleCommand>(
		std::string(this->ros_namespace + "/fmu/in/vehicle_command"), 10);
	this->offboard_control_mode_pub = this->attached_node->create_publisher<OffboardControlMode>(
		std::string(this->ros_namespace + "/fmu/in/offboard_control_mode"), 10);
	/*Creating subscriptions*/
	this->vehicle_status_sub = this->attached_node->create_subscription<VehicleStatus>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_status"), 10, 
		std::bind(& Vehicle::vehicle_status_cb, this, _1));
	this->vehicle_attitude_sub = this->attached_node->create_subscription<VehicleAttitude>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_attitude"), 10, 
		std::bind(& Vehicle::vehicle_attitude_cb, this, _1));
	this->vehicle_control_mode_sub = this->attached_node->create_subscription<VehicleControlMode>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_control_mode"), 10, 
		std::bind(& Vehicle::vehicle_control_mode_cb, this, _1));
	this->timesync_status_sub = this->attached_node->create_subscription<TimesyncStatus>(
		std::string(this->ros_namespace + "/fmu/out/timesync_status"), 10, 
		std::bind(& Vehicle::timesync_status_cb, this, _1));
	
	RCLCPP_INFO(this->attached_node->get_logger(), "Vehicle initialized");
}

void Vehicle::send_arm_command(){
	/* Send arm command to the drone via DDS */
	VehicleCommand command_msg = this->create_vehicle_command(
		VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	this->vehicle_command_pub->publish(command_msg);
}

void Vehicle::send_disarm_command(){
	VehicleCommand command_msg = this->create_vehicle_command(
		VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	this->vehicle_command_pub->publish(command_msg);
}

VehicleCommand Vehicle::create_vehicle_command(int command, float param1, float param2){
	/*Create vehicle command message*/
	auto msg = VehicleCommand();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;  // command ID
	msg.target_system = this->system_id();  // system which should execute the command
	msg.target_component = this->component_id(); // component which should execute the command, 0 for all components
	msg.source_system = 10; // system sending the command
	msg.source_component = 1; // component sending the command
	msg.from_external = true;
	msg.timestamp = this->get_now_timestamp(); // time in microseconds
	return msg;
}

void Vehicle::offboard_flight_mode() {
	/*Change to offboard flight mode*/
	VehicleCommand command_msg = this->create_vehicle_command(
		VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
	this->vehicle_command_pub->publish(command_msg);
}

void Vehicle::publish_offboard_control_signal(){
	// This message have to be published > 2 Hz
	this->offboard_control_mode->timestamp = this->get_now_timestamp();
	this->offboard_control_mode_pub->publish(*this->offboard_control_mode);
}

void Vehicle::publish_trajectory_setpoint(px4_msgs::msg::TrajectorySetpoint message){
	this->trajectory_setpoint_pub->publish(message);
}
