
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vehicle.hpp"
#include "pilot.hpp"

using namespace std::chrono_literals;

class DroneTrackerNode : public rclcpp::Node
{
  public:
    DroneTrackerNode() : Node("drone_tracker"){
      this->drone = std::make_shared<Vehicle>(new Vehicle("", this->shared_from_this()));

      // this->pilot = std::make_shared<Pilot>(Pilot());
      // this->pilot->set_vehicle(this->drone);
       
      // this->offboard_timer = this->create_wall_timer( 100ms, std::bind(&Pilot::offboard_callback, this->pilot));
      // this->mission_timer = this->create_wall_timer( 500ms, std::bind(&Pilot::mission_update, this->pilot));
    }

  private:
    std::shared_ptr<Pilot> pilot;
    std::shared_ptr<Vehicle> drone;
    rclcpp::TimerBase::SharedPtr offboard_timer;
    rclcpp::TimerBase::SharedPtr mission_timer;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
