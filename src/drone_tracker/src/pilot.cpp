#include "rclcpp/rclcpp.hpp"
#include "pilot.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace px4_msgs::msg;

Pilot::Pilot(){
	this->current_state = IDLE;
	this->next_state = IDLE;
}

void Pilot::mission_update(){
	if(this->drone == nullptr)
		return;
	
	current_state = next_state;

	switch(current_state){
		case IDLE:
			if(this->drone->nav_state() == VehicleStatus::NAVIGATION_STATE_OFFBOARD){
				RCLCPP_INFO(this->drone->attached_node->get_logger(), "IDLE=>PREFLIGHT_CHECK");
				next_state = PREFLIGHT_CHECK;
			}else{
				this->drone->offboard_flight_mode(); /*Set offboard flight mode*/
				next_state = IDLE;
			}
			break;
		//-----------------------------------------------------
		case PREFLIGHT_CHECK:
			if(this->drone->preflight_check())	{
				next_state = ARMING;
				RCLCPP_INFO(this->drone->attached_node->get_logger(), "PREFLIGHT_CHECK=>ARMING");}
			else {next_state = PREFLIGHT_CHECK;}
		break;
		//-----------------------------------------------------
		case ARMING:
			this->drone->send_arm_command();
			next_state = ARMED;
			RCLCPP_INFO(this->drone->attached_node->get_logger(), "ARMING=>ARMED");
			break;
		//-----------------------------------------------------
		case ARMED:
			if(this->drone->is_armed()){
				next_state = MISSION;
				RCLCPP_INFO(this->drone->attached_node->get_logger(), "ARMED=>MISSION");}
			else{next_state = ARMED;}
			break;
		//-----------------------------------------------------
		case MISSION:
			if(this->mission_func()){next_state = LANDING;}
			else{next_state = MISSION;}
			break;
		//-----------------------------------------------------
		case LANDING:
			break;
		//-----------------------------------------------------
		case FAILSAFE:
			break;

		
		default:
			next_state = IDLE;
			break;
	}

}

bool Pilot::mission_func(){
	TrajectorySetpoint msg = TrajectorySetpoint();
	msg.position = {0.0 , 0.0, -4.0};
	msg.timestamp = this->drone->get_now_timestamp();
	msg.yaw = 3.14/2.0;
	this->drone->publish_trajectory_setpoint(msg);
	return true;
}

