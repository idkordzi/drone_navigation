#include "pid_controller.hpp"


namespace DRONE_NAVIGATION {

PID::PID(float Kp, float Ki, float Kd, float min_lim, float max_lim)
: Kp_(Kp), Ki_(Ki), Kd_(Kd), out_min_(min_lim), out_max_(max_lim)
{}

float PID::calculate(float sp, float pv) {
  std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->ts_;
  float output = this->calculate(sp, pv, time_passed.count());
  this->ts_ = std::chrono::system_clock::now();
  return output;
}

float PID::calculate(float sp, float pv, float dt) {
  dt = clip(dt, this->config_.min_dt_, this->config_.max_dt_);

  float error = sp - pv;
  float Pout = this->Kp_ * error;

  this->mem_int_ += error * dt;
  float Iout = this->Ki_ * this->mem_int_;

  float deriv = (error - this->mem_err_) / dt;
  float Dout = this->Kd_ * deriv;

  float output = clip(Pout + Iout + Dout, this->out_min_, this->out_max_);
  this->mem_err_ = error;

  return output;
}

} // namespace DRONE_NAVIGATION
