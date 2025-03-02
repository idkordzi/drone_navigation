#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include "pid_controller.hpp"


namespace DRONE_NAVIGATION {

struct FlightControllerConfig {

  float hover_vel_fr_ = 0.0f;
  float hover_vel_fl_ = 0.0f;
  float hover_vel_rr_ = 0.0f;
  float hover_vel_rl_ = 0.0f;

  float reg_up_Kp_ = 0.0f;
  float reg_up_Ki_ = 0.0f;
  float reg_up_Kd_ = 0.0f;

  float reg_vel_Kp_ = 0.0f;
  float reg_vel_Ki_ = 0.0f;
  float reg_vel_Kd_ = 0.0f;

  float reg_rot_Kp_ = 0.0f;
  float reg_rot_Ki_ = 0.0f;
  float reg_rot_Kd_ = 0.0f;
};

class FlightController {

public:

  FlightController();
  ~FlightController() = default;

  void setGoal(Eigen::Vector3f goal_position);

  std::vector<float> calculateControl(Eigen::Vector3f position, Eigen::Vector3f orientation);
  
private:

  FlightControllerConfig config_ = {};

  Eigen::Vector3f goal_position_ = {};

  PID reg_up_;
  PID reg_vel_;
  PID reg_rot_;
};

} // namespace DRONE_NAVIGATION
