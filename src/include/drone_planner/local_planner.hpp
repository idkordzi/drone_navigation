#pragma once

#include <chrono>
#include "Eigen/Dense"
#include "common.hpp"
#include "config.hpp"
#include "polar_histogram.hpp"
#include "point_cloud.hpp"


namespace DRONE_NAVIGATION {

class LocalPlanner {

public:

  LocalPlanner();
  ~LocalPlanner() = default;

  void setFOV(const FOV& fov);
  void setPointCloud(const PointCloud<PointXYZ>& cloud);
  void setGoal(const Eigen::Vector3f& goal);
  
  Eigen::Vector3f getNext() const;

  void setPosition(Eigen::Vector3f position);
  void setOrientation(Eigen::Vector3f orientation);
  void setVelocity(Eigen::Vector3f velocity);

protected:

  void processPointCloud();
  void generateHistogram(PolarHistogram& histogram, const Eigen::Vector3f& position);

  LocalPlannerConfig local_planner_config_ = {};

  Eigen::Vector3f position_     = Eigen::Vector3f::Zero(); // drone position in odom frame
  Eigen::Vector3f lin_velocity_ = Eigen::Vector3f::Zero(); // drone linear velocity in odom frame
  Eigen::Vector3f orientation_  = Eigen::Vector3f::Zero(); // drone orientation in odom frame
  // Eigen::Vector3f ang_velocity_ = Eigen::Vector3f::Zero(); // drone angular velocity in odom frame

  bool goal_updated_ = false;
  Eigen::Vector3f goal_      = Eigen::Vector3f::Zero();
  Eigen::Vector3f prev_goal_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f next_      = Eigen::Vector3f::Zero();

  PointCloud<PointXYZI> base_cloud_ = {};

  bool fov_set_ = false;
  FOV fov_cache_ = {};

  bool cloud_set_ = false;
  PointCloud<PointXYZ> cloud_cache_ = {};

  std::chrono::system_clock::time_point last_processing_time_;

};

} // namespace DRONE_NAVIGATION
