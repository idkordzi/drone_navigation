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
  FlightControllerTestClass() = default;
  ~FlightControllerTestClass() = default;

  void runTests() {

    Eigen::Vector3f goal = {0.0, 0.0, 2.0};

    this->setGoal(goal);

    Eigen::Vector3f position    = {0.0, 0.0, 1.0};
    Eigen::Vector3f orientation = {0.0, 0.0, 0.0};

    std::vector<float> rotor_control = this->calculateControl(position, orientation);

    for (const auto& ctrl : rotor_control) std::cout << ctrl << std::endl;

    std::cout << "[INFO] Drone Controller: OK\n";
  }

};

} // namespace DRONE_NAVIGATION


int main() {

  DRONE_NAVIGATION::FlightControllerTestClass flight_controller_test_class = DRONE_NAVIGATION::FlightControllerTestClass();
  flight_controller_test_class.runTests();

  return 0;
}