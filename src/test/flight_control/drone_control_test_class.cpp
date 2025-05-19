#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "flight_controller.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace DRONE_NAVIGATION {

class FlightControllerTestClass : public FlightController {

public:
  FlightControllerTestClass() {}
  FlightControllerTestClass(FlightControllerConfig config) : FlightController(config) {}
  ~FlightControllerTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_fligth_controller_test'\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: standstill\n";

    Eigen::Vector3f goal, position, orientation, velocity;
    std::vector<float> rotor_control;

    goal = Eigen::Vector3f(0.0, 0.0, 0.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(0.0, 0.0, 0.0);
    velocity = Eigen::Vector3f(0.0, 0.0, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == 0 && 
        rotor_control[1] == 0 && 
        rotor_control[2] == 0 && 
        rotor_control[3] == 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: moving upward\n";

    goal = Eigen::Vector3f(0.0, 0.0, 1.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(0.0, 0.0, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == rotor_control[1] && 
        rotor_control[0] == rotor_control[2] && 
        rotor_control[0] == rotor_control[3] && 
        rotor_control[0] > 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: pitch change\n";

    goal = Eigen::Vector3f(0.0, 0.0, 0.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(0.0, -0.1, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == rotor_control[1] && 
        rotor_control[0] < 0 && 
        rotor_control[2] == rotor_control[3] && 
        rotor_control[2] > 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: roll change\n";

    goal = Eigen::Vector3f(0.0, 0.0, 0.0);
    position = Eigen::Vector3f(0.0, 0.0, 0.0);
    orientation = Eigen::Vector3f(-0.1, 0.0, 0.0);

    this->resetPID();
    this->setGoal(goal);
    rotor_control = this->calculateControl(position, orientation, velocity);

    if (rotor_control[0] == rotor_control[2] && 
        rotor_control[0] < 0 && 
        rotor_control[1] == rotor_control[3] && 
        rotor_control[1] > 0)
      std::cout << "[SUCCESS] Rotor control: OK\n";
    else
      std::cout << "[ERROR] Bad rotor contorls\n";
    std::cout << "Print rotor control:\n";
    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    std::cout << "[INFO] Test STOP\n";
  }
};

} // namespace DRONE_NAVIGATION


int main() {

  DRONE_NAVIGATION::FlightControllerConfig flight_controller_config;
  flight_controller_config.reg_mov_z_Kp = 1.0;
  flight_controller_config.reg_rot_x_Kp = 1.0;
  flight_controller_config.reg_rot_y_Kp = 1.0;
  flight_controller_config.reg_rot_z_Kp = 1.0;
  flight_controller_config.pid_min = -1.0;
  flight_controller_config.pid_max =  1.0;

  DRONE_NAVIGATION::FlightControllerTestClass flight_controller_test_class = DRONE_NAVIGATION::FlightControllerTestClass(flight_controller_config);
  flight_controller_test_class.runTest();

  return 0;
}