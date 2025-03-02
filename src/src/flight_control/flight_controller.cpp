#include "flight_controller.hpp"


namespace DRONE_NAVIGATION {

FlightController::FlightController() 
: reg_up_(this->config_.reg_up_Kp_, this->config_.reg_up_Ki_, this->config_.reg_up_Kd_),
  reg_vel_(this->config_.reg_vel_Kp_, this->config_.reg_vel_Ki_, this->config_.reg_vel_Kd_),
  reg_rot_(this->config_.reg_rot_Kp_, this->config_.reg_rot_Ki_, this->config_.reg_rot_Kd_)
{}

void FlightController::setGoal(Eigen::Vector3f goal_position) {
  this->goal_position_ = goal_position;
}

std::vector<float> FlightController::calculateControl(Eigen::Vector3f position, Eigen::Vector3f orientation) {

  std::vector<float> control_vector = {
    this->config_.hover_vel_fr_, // front right
    this->config_.hover_vel_fl_, // front left
    this->config_.hover_vel_rr_, // rear right
    this->config_.hover_vel_rl_, // rear left
  };

  Eigen::Vector3f pos_diff = this->goal_position_ - position;

  float slope = (this->goal_position_.y() - position.y()) / (this->goal_position_.x() - position.x());
  float yaw_diff = std::atan(slope) - orientation.z();

  // Calculate vertical velocity
  float ctr_up = this->reg_up_.calculate(this->goal_position_.z(), position.z());
  control_vector[0] += ctr_up;
  control_vector[1] += ctr_up;
  control_vector[2] += ctr_up;
  control_vector[3] += ctr_up;

  // Calculate horizontal velocity
  // TODO include full horizontal movement (now is only forward/backward)
  float ctr_vel = this->reg_vel_.calculate(0.0f, pos_diff.head<2>().norm());
  control_vector[0] += -ctr_vel;
  control_vector[1] += -ctr_vel;
  control_vector[2] +=  ctr_vel;
  control_vector[3] +=  ctr_vel;

  // Calculate angular velocity
  float ctr_rot = this->reg_rot_.calculate(0.0f, yaw_diff);
  control_vector[0] +=  ctr_rot;
  control_vector[1] += -ctr_rot;
  control_vector[2] += -ctr_rot;
  control_vector[3] +=  ctr_rot;

  return control_vector;
}

} // namespace DRONE_NAVIGATION
