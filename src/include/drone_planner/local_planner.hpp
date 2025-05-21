#pragma once

#include <chrono>
#include "Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "common.hpp"
#include "polar_histogram.hpp"
#include "point_cloud.hpp"


namespace DRONE_NAVIGATION {

struct LocalPlannerConfig {

  // general params
  float thread_freq = 10.0; // [Hz]

  bool skip_planning = false;

  // camera params
  float sensor_min_range = 0.2f;  // [m]
  float sensor_max_range = 12.0f; // [m]

  float camera_fov_h = 86.0f; // [deg]
  float camera_fov_v = 58.0f; // [deg]

  // polar histogram params
  int alpha = 6; // [deg]

  // point cloud params
  float point_max_age = 5.0f; // [s]

  // goal settings
  float goal_dev_margin = 0.1f; // [m]

  unsigned prev_goal_num = 3;
  unsigned extr_goal_num = 1;

  float goal_min_dist = 1.0f; // [m]
  float goal_min_alt_diff = 0.5; // [m] , cannot be larger than 'goal_min_dist_'

  // trajectory planning
  unsigned max_candidates_per_it = 3;
  float drone_pos_margin = 0.1f; // [m]
  float planning_step = 1.0f; // [m]
};

struct CostParams {
  float yaw_cost_param      = 0.5f;
  float pitch_cost_param    = 3.0f;
  float velocity_cost_param = 1.5f;
  float obstacle_cost_param = 5.0f;
};

struct CostFunctionOutput {
  CostFunctionOutput() : distance_cost(0.0f), state_cost(0.0f) {}
  CostFunctionOutput(float d, float s) : distance_cost(d), state_cost(s) {}

  float distance_cost = 0.0f;
  float state_cost = 0.0f;
};

struct MoveDirection {
  MoveDirection(float _elev, float _azim, float _cost) : elevation(_elev), azimuth(_azim), cost(_cost) {}

  bool operator<(const MoveDirection& d) const { return this->cost < d.cost;}
  bool operator>(const MoveDirection& d) const { return this->cost > d.cost;}
  
  float elevation = 0.0f;
  float azimuth = 0.0f;
  float cost = 0.0f;
};

class LocalPlanner {

public:

  LocalPlanner();
  LocalPlanner(LocalPlannerConfig config);
  ~LocalPlanner() = default;

  void setState(Eigen::Vector3f position,
                Eigen::Vector3f orientation,
                Eigen::Vector3f velocity);
  
  void setGoal(Eigen::Vector3f goal);

  void setPointCloud(const PointCloud<PointXYZ>& cloud);

  void run();

  Eigen::Vector3f getNext() const;
  
  cv::Mat getHistImage() const;
  cv::Mat getCostImage() const;

  void reset();

protected:

  void initialize();

  PointXYZ transformPoint(PointXYZ point) const;
  void processPointCloud();

  CostFunctionOutput costFunction(const PolarPoint& candidate,
                                  const Eigen::Vector3f& position,
                                  const Eigen::Vector3f& velocity,
                                  float obstacle_distance) const;
  void getCostMatrix(const PolarHistogram& histogram,
                     const Eigen::Vector3f& position,
                     const Eigen::Vector3f& velocity,
                     Eigen::MatrixXf& cost_matrix,
                     cv::Mat& cost_image) const;
  void getBestMoveDirections(const Eigen::MatrixXf& cost_matrix,
                             std::vector<MoveDirection>& direction_list) const;
  void planNext();

  void generateHistImage(const PolarHistogram& histogram,
                         cv::Mat& image_data) const;
  void generateCostImage(const Eigen::MatrixXf& cost_matrix,
                         const Eigen::MatrixXf& distance_matrix,
                         cv::Mat& image_data) const;
  
  LocalPlannerConfig config_ = {};
  CostParams cost_params_ = {};

  bool drone_ready_ = false;

  bool goal_updated_ = false;
  Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal_pos_ = Eigen::Vector3f::Zero();
  
  std::vector<Eigen::Vector3f> prev_goal_array_ = {};
  std::vector<Eigen::Vector3f> extr_goal_array_ = {};

  bool state_updated_ = false;
  Eigen::Vector3f position_     = Eigen::Vector3f::Zero(); // drone position in odom frame
  Eigen::Vector3f orientation_  = Eigen::Vector3f::Zero(); // drone orientation in odom frame
  Eigen::Vector3f lin_velocity_ = Eigen::Vector3f::Zero(); // drone linear velocity in odom frame

  Eigen::Vector3f prev_position_ = Eigen::Vector3f::Zero();

  Eigen::Matrix3f rotation_matrix_ = Eigen::Vector3f::Ones().asDiagonal();
  float translation_flatten_[12] = {};
  FOV fov_ = {};

  bool cloud_updated_ = false;
  PointCloud<PointXYZ> cloud_cache_ = {};
  std::chrono::system_clock::time_point last_processing_time_;

  PolarHistogram histogram_ = {};
  
  Eigen::Vector3f next_ = Eigen::Vector3f::Zero();

  cv::Mat hist_image_ = {};
  cv::Mat cost_image_ = {};
};

} // namespace DRONE_NAVIGATION
