#include "local_planner.hpp"


namespace DRONE_NAVIGATION {

LocalPlanner::LocalPlanner() {
  this->last_processing_time_ = std::chrono::system_clock::now();
};

void LocalPlanner::setFOV(const FOV& fov) {
  this->fov_cache_ = FOV(fov.yaw_deg, fov.pitch_deg, fov.h_fov_deg, fov.v_fov_deg);
  this->fov_set_ = true;
}

void LocalPlanner::setPointCloud(const PointCloud<PointXYZ>& cloud) {
  this->cloud_cache_.cloud_.clear();
  for (auto point : cloud) {
    this->cloud_cache_.cloud_.push_back(point);
  }
  this->cloud_set_ = true;
}

void LocalPlanner::setGoal(const Eigen::Vector3f& goal) {
  if ((this->goal_-goal).norm() > this->local_planner_config_.goal_dev_margin_) {
    this->prev_goal_ = this->goal_;
    this->goal_ = Eigen::Vector3f(goal.x(), goal.y(), goal.z());
    this->goal_updated_ = true;
  }
  this->goal_updated_ = false;
}

Eigen::Vector3f LocalPlanner::getNext() const {
  return this->next_;
}

void LocalPlanner::setPosition(Eigen::Vector3f position) {
  this->position_ = position;
}

void LocalPlanner::setOrientation(Eigen::Vector3f orientation) {
  this->orientation_ = orientation;
}

void LocalPlanner::setVelocity(Eigen::Vector3f velocity) {
  this->lin_velocity_ = velocity;
}

void LocalPlanner::processPointCloud()
{
  if (!this->fov_set_ || !this->cloud_set_) {
    // throw error/warning if new data is not available
  }

  std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->last_processing_time_;
  double elapsed = time_passed.count();

  PointCloud<PointXYZI> updated_cloud;

  for (const PointXYZ& point : this->cloud_cache_) {
    if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
      float distanceSq = (this->position_ - toEigen(point)).squaredNorm();
      if (sqr(this->local_planner_config_.sensor_min_range_) < distanceSq && 
          distanceSq < sqr(this->local_planner_config_.sensor_max_range_))
        updated_cloud.cloud_.push_back(PointXYZI(point.x, point.y, point.z, 0.0));
    }
  }

  for (const PointXYZI& point : this->base_cloud_) {
    float distanceSq = (this->position_ - toEigen(point)).squaredNorm();
    if (distanceSq < std::sqrt(this->local_planner_config_.sensor_max_range_)) {
      PolarPoint polar = convertCartesianToPolar(toEigen(point), this->position_);
      if (point.i < this->local_planner_config_.point_max_age_ && !pointInsideFOV(this->fov_cache_, polar))
        updated_cloud.cloud_.push_back(PointXYZI(point.x, point.y, point.z, point.i+elapsed));
    }
  }

  std::swap(this->base_cloud_, updated_cloud);
  this->last_processing_time_ = std::chrono::system_clock::now();

  this->fov_set_   = false;
  this->cloud_set_ = false;
}

void LocalPlanner::generateHistogram(PolarHistogram& histogram, const Eigen::Vector3f& position) {
  Eigen::MatrixXi counter(ELEVATION_RESOLUTION, AZIMUTH_RESOLUTION);
  counter.fill(0);
  for (auto point : this->base_cloud_) {
    PolarPoint polar = convertCartesianToPolar(toEigen(point), position);
    float dist = polar.radi;
    Eigen::Vector2i idx = convertPolarToHistogramIndex(polar, CELL_SIZE);
    counter(idx.y(), idx.x())++;
    histogram.setDistance(idx.y(), idx.x(), histogram.getDistance(idx.y(), idx.x()) + dist);
  }

  // Normalize and get mean in distance bins
  for (int elev = 0; elev < ELEVATION_RESOLUTION; elev++) {
    for (int azim = 0; azim < AZIMUTH_RESOLUTION; azim++) {
      if (counter(elev, azim) > 0) histogram.setDistance(elev, azim, histogram.getDistance(elev, azim) / counter(elev, azim));
      else histogram.setDistance(elev, azim, 0.0f);
    }
  }
}

} // namespace DRONE_NAVIGATION
