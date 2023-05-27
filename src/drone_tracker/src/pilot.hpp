#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "vehicle.hpp"

enum MissionState {
	IDLE,
	PREFLIGHT_CHECK,
	ARMING,
	ARMED,
	MISSION,
	LANDING,
	FAILSAFE
};


class Pilot {

public:
	Pilot();

	void set_vehicle(std::shared_ptr<Vehicle> drone){this->drone = drone;};
	void mission_update();
	void offboard_callback(){this->drone->publish_offboard_control_signal();};

private:
	bool mission_func();
	std::shared_ptr<Vehicle> drone = nullptr;
	MissionState current_state, next_state;


};