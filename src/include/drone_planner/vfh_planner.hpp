#pragma once

#include <chrono>
#include "Eigen/Dense"
#include "star_planner.hpp"


namespace DRONE_NAVIGATION {

class VFHPlanner : public StarPlanner {

public:

  VFHPlanner();
  ~VFHPlanner() = default;

  void updateStatus();
  void run();
  uint8_t getStatus() const {return this->planner_state_;}
  Eigen::Vector3f getNextGoal() const {return this->next_;}

private:

  VFHPlannerConfig vhf_planner_config_ = {};

  /**
   * Planner state:
   *  0 - unintialized
   *  1 - initialized, no trajectory available/ goal has changed
   *  2 - initialized, trajectory available
   */
  uint8_t planner_state_ = 0;
};

} // namespace DRONE_NAVIGATION
