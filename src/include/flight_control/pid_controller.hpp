#pragma once

#include <cmath>
#include <chrono>


namespace DRONE_NAVIGATION {

inline float clip(float val, float lo_lim = -INFINITY, float hi_lim = INFINITY) {
  return std::min(std::max(val, lo_lim), hi_lim);
}

struct PIDConfig {
  float min_dt_ = 1e-15; // [s]
  float max_dt_ = 1.0;   // [s]
};

class PID {

public:

  PID(float Kp, float Ki, float Kd, float min_lim = -INFINITY, float max_lim = INFINITY);
  ~PID() = default;

  float calculate(float sp, float pv);
  float calculate(float sp, float pv, float dt);

private:

  PIDConfig config_ = {};

  float Kp_ = 0.0f;
  float Ki_ = 0.0f;
  float Kd_ = 0.0f;

  float mem_err_ = 0.0f;
  float mem_int_ = 0.0f;

  float out_min_ = -INFINITY;
  float out_max_ = INFINITY;

  std::chrono::system_clock::time_point ts_ = {};
};

} // namespace DRONE_NAVIGATION
