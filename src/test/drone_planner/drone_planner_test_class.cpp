#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "vfh_planner.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace DRONE_NAVIGATION {

class VFHPlannerTestClass : public VFHPlanner {

public:
  VFHPlannerTestClass() = default;
  ~VFHPlannerTestClass() = default;

  void runTests() {

    Eigen::Vector3f position    = Eigen::Vector3f::Zero();
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity    = Eigen::Vector3f::Zero();

    FOV fov = FOV();
    Eigen::Vector3f goal = Eigen::Vector3f::Zero();

    PointCloud<PointXYZ> cloud = {};

    this->setFOV(fov);
    this->setGoal(goal);

    this->setPointCloud(cloud);

    this->setPosition(position);
    this->setOrientation(orientation);
    this->setVelocity(velocity);

    std::cout << "[INFO] Local Planner: OK\n";

    this->updateStatus();
    unsigned status = this->getStatus();

    std::cout << "status: (" << status << ")\n";

    Eigen::Vector3f next_goal = this->getNextGoal();

    std::cout << next_goal.x() << " " << next_goal.y() << " " << next_goal.z() << "\n";

    std::cout << "[INFO] VFH Planner: OK\n";
  }

};

} // namespace DRONE_NAVIGATION


int main() {

  DRONE_NAVIGATION::VFHPlannerTestClass vfh_planner_test_class = DRONE_NAVIGATION::VFHPlannerTestClass();
  vfh_planner_test_class.runTests();

  return 0;
}