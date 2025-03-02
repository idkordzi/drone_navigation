#pragma once

#include <cmath>


namespace DRONE_NAVIGATION {

struct LocalPlannerConfig {
  float sensor_min_range_ = 0.2f;  // [m]
  float sensor_max_range_ = 12.0f; // [m]
  float point_max_age_    = 10.0f; // [s]

  float camera_fov_h = 0.0f; // [deg]
  float camera_fov_v = 0.0f; // [deg]

  float goal_dev_margin_ = 0.05; // [m]
};

struct StarPlannerConfig {
  int children_per_node_  = 1;
  int max_expanded_nodes_ = 5;

  float node_step_distance_   = 1.0f;  // [m]
  float node_distance_margin_ = 0.2f;  // [m]
  float max_path_length_      = 4.0f;
  float smoothing_margin_     = 30.0f; // [deg]
  float heuristic_weight_     = 10.0f;

  // cost parameters
  float yaw_cost_param      = 0.5f;
  float pitch_cost_param    = 3.0f;
  float velocity_cost_param = 1.5f;
  float obstacle_cost_param = 5.0f;
};

struct VFHPlannerConfig {

  float drone_pos_margin_ = 0.2f; // [m]
};

struct ModelParameters {
  int param_mpc_auto_mode = -1; // Auto sub-mode - 0: default line tracking, 1 jerk-limited trajectory

  float param_mpc_jerk_min     = NAN; // Velocity-based minimum jerk limit
  float param_mpc_jerk_max     = NAN; // Velocity-based maximum jerk limit
  float param_acc_up_max       = NAN; // Maximum vertical acceleration in velocity controlled modes upward
  float param_mpc_z_vel_max_up = NAN; // Maximum vertical ascent velocity
  float param_mpc_acc_down_max = NAN; // Maximum vertical acceleration in velocity controlled modes down
  float param_mpc_acc_hor      = NAN; // Maximum horizontal acceleration for auto mode and
                                      // maximum deceleration for manual mode
  
  float param_mpc_xy_cruise  = NAN; // Desired horizontal velocity in mission
  float param_mpc_tko_speed  = NAN; // Takeoff climb rate
  float param_mpc_land_speed = NAN; // Landing descend rate

  float param_nav_acc_rad = NAN;

  // TODO: add estimator limitations for max speed and height

  float param_cp_dist = NAN; // Collision Prevention distance to keep from obstacle. -1 for disabled
};

} // namespace DRONE_NAVIGATION