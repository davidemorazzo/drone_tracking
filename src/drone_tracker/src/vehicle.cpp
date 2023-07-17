#include "vehicle.hpp"
#include "rclcpp/rclcpp.hpp"
#include <math.h>

using namespace px4_msgs::msg;
using std::placeholders::_1;

Vehicle::Vehicle() : Node("vehicle_node"){
	if (std::string(this->get_namespace()) == "/"){
		this->ros_namespace = "";
	}else{
		this->ros_namespace = this->get_namespace();
	}
	
	/*Offboard control mode message*/
	this->offboard_control_mode = std::make_shared<OffboardControlMode>();
	this->offboard_control_mode->timestamp = this->get_now_timestamp();
	this->offboard_control_mode->body_rate = false;
	this->offboard_control_mode->attitude = false;
	this->offboard_control_mode->position = true;
	this->offboard_control_mode->velocity = false;
	this->offboard_control_mode->acceleration = false;

	/*Creating publishers*/
	this->vehicle_command_pub = this->create_publisher<VehicleCommand>(
		std::string(this->ros_namespace + "/fmu/in/vehicle_command"), pub_qos);
	this->offboard_control_mode_pub = this->create_publisher<OffboardControlMode>(
		std::string(this->ros_namespace + "/fmu/in/offboard_control_mode"), pub_qos);
	this->trajectory_setpoint_pub = this->create_publisher<TrajectorySetpoint>(
		std::string(ros_namespace + "/fmu/in/trajectory_setpoint"), pub_qos);

	/*Creating subscriptions*/
	this->vehicle_status_sub = this->create_subscription<VehicleStatus>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_status"), sub_qos, 
		std::bind(& Vehicle::vehicle_status_cb, this, _1));
	this->vehicle_control_mode_sub = this->create_subscription<VehicleControlMode>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_control_mode"), sub_qos, 
		std::bind(& Vehicle::vehicle_control_mode_cb, this, _1));
	this->timesync_status_sub = this->create_subscription<TimesyncStatus>(
		std::string(this->ros_namespace + "/fmu/out/timesync_status"), sub_qos, 
		std::bind(& Vehicle::timesync_status_cb, this, _1));
	this->vehicle_odometry_sub = this->create_subscription<VehicleOdometry>(
		std::string(this->ros_namespace + "/fmu/out/vehicle_odometry"), sub_qos, 
		std::bind(& Vehicle::vehicle_odometry_cb, this, _1));
	this->create_subscription<std_msgs::msg::Float64MultiArray>(
		std::string(this->ros_namespace + "/acceleration_cmd"), sub_qos, 
		std::bind(& Vehicle::estimator_cb, this, _1));

	this->current_state = MissionState::IDLE;
	this->next_state = MissionState::IDLE;

	RCLCPP_INFO(this->get_logger(), "Vehicle initialized");
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
	msg.command = command;  					// command ID
	msg.target_system = this->system_id();  	// system which should execute the command
	msg.target_component = this->component_id();// component which should execute the command, 0 for all components
	msg.source_system = 10; 					// system sending the command
	msg.source_component = 1; 					// component sending the command
	msg.from_external = true;
	msg.timestamp = this->get_now_timestamp(); 	// time in microseconds
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

bool Vehicle::xrce_connected(){
	return (this->vehicle_status != nullptr) &&
		(this->vehicle_control_mode != nullptr) &&
		(this->timesync_status != nullptr) &&
		(this->vehicle_odometry != nullptr);
}

void Vehicle::mission_update(){

	current_state = next_state;

	switch(current_state){
		case IDLE:
			if (xrce_connected()){
				if(this->nav_state() == VehicleStatus::NAVIGATION_STATE_OFFBOARD){
					RCLCPP_INFO(this->get_logger(), "IDLE=>PREFLIGHT_CHECK");
					next_state = MissionState::PREFLIGHT_CHECK;
				}else{
					this->offboard_flight_mode(); /*Set offboard flight mode*/
					next_state = MissionState::IDLE;
				}
			}
			break;
		//-----------------------------------------------------
		case PREFLIGHT_CHECK:
			if(this->preflight_check())	{
				if(!this->is_armed()){
					this->vehicle_starting_position[0] = this->vehicle_odometry->position[0];
					this->vehicle_starting_position[1] = this->vehicle_odometry->position[1];
					this->vehicle_starting_position[2] = this->vehicle_odometry->position[2];
					this->send_arm_command();
					next_state = MissionState::MISSION;
					RCLCPP_INFO(this->get_logger(), "PREFLIGHT_CHECK=>MISSION");
					}
				}
			else 
				{next_state = MissionState::PREFLIGHT_CHECK;}
		break;
		//-----------------------------------------------------
		case MISSION:
			if(this->mission_func())
			{
				RCLCPP_INFO(get_logger(), "MISSION=>LANDING");
				next_state = MissionState::LANDING;
			}
			else
				{next_state = MissionState::MISSION;}
			break;
		//-----------------------------------------------------
		case LANDING:
			if(vehicle_status->nav_state != VehicleStatus::NAVIGATION_STATE_AUTO_LAND){
				vehicle_command_pub->publish(
					create_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND)
				);
			}
			break;
		//-----------------------------------------------------

		
		default:
			next_state = MissionState::IDLE;
			break;
	}
}

void Vehicle::publish_hor_acc_setpoint(float acc_x, float acc_y, float height, float yaw){
	/* Control drone in acceleration in XY and set a height */
	TrajectorySetpoint msg = TrajectorySetpoint();
	offboard_control_mode->acceleration=true;
	offboard_control_mode->velocity = false;
	offboard_control_mode->position = false;
	msg.position = {NAN, NAN, height};				// Important set the correct NAN values
	msg.velocity = {NAN, NAN, NAN};
	msg.acceleration = {acc_x, acc_y, NAN};
	msg.timestamp = this->get_now_timestamp();
	msg.yaw = yaw;
	trajectory_setpoint_pub->publish(msg);
}

void Vehicle::publish_pos_setpoint(float x, float y, float z, float yaw){
	TrajectorySetpoint msg = TrajectorySetpoint();
	offboard_control_mode->position = true;
	msg.position = {
		x + this->vehicle_starting_position[0], 
		y + this->vehicle_starting_position[1], 
		z};
	msg.timestamp = this->get_now_timestamp();
	msg.yaw = yaw;
	trajectory_setpoint_pub->publish(msg);
}

bool Vehicle::mission_func(){
	if (mission_cb_cnt < 30){
		publish_pos_setpoint(0.0, 0.0, -2, 0);
	}else{
		// publish_pos_setpoint(0.3, 0.3, -2, 0);
		/* Enable flocking algorithm command */
		publish_hor_acc_setpoint(this->flocking_ax, this->flocking_ay, -2.0, 0);
	}
	mission_cb_cnt++;
	return false;
}

/* When a new acceleration command is available from the estimator, it is published to the autopilot */
void Vehicle::estimator_cb(const std_msgs::msg::Float64MultiArray & message){
	// ay is inverted to manage FLU=>FRD transform
	this->flocking_ax = message.data[0];
	this->flocking_ay = -message.data[1];
}