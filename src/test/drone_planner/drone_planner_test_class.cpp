#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "local_planner.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace DRONE_NAVIGATION {

inline bool eqmarg(float a, float b, float m) {
  return std::abs(a-b) <= m;
}

class PolarHistogramTestClass {

public:
  PolarHistogramTestClass() = default;
  ~PolarHistogramTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_polar_histogram_test'\n";

    PolarHistogram histogram = PolarHistogram(6); // cell size of 6 degrees

    if (histogram.isEmpty())
      std::cout << "[SUCCESS] Initialization with (6) [deg] cell size\n";
    else
      std::cout << "[ERROR] Histogram not empty on initialization\n";
    
    histogram.setDistance(0, 0, 1.0f);

    if (eqmarg(histogram.getDistance(0, 0), 1.0f, 0.0f))
      std::cout << "[SUCCESS] Successfully wrote and read from histogram at (0, 0) [deg] (elevetion, azimuth)\n";
    else
      std::cout << "[ERROR] When reading from cell at (0, 0) [deg] (elevetion, azimuth): '" << histogram.getDistance(0, 0) << "' =/= 1.0\n";

    Eigen::Vector2i h_idx = convertAngleToHistogramIndex(61.0f, 121.0f, 6);
    histogram.setDistance(h_idx.y(), h_idx.x(), 1.0f);

    if (eqmarg(histogram.getDistance(h_idx.y(), h_idx.x()), 1.0f, 0.0f))
      std::cout << "[SUCCESS] Successfully wrote and read from histogram at (61, 121) [deg] (elevetion, azimuth)\n";
    else
      std::cout << "[ERROR] When reading from cell at (61, 121) [deg] (elevetion, azimuth): '" << histogram.getDistance(h_idx.y(), h_idx.x())
                << "' =/= 1.0\n";

    std::cout << "[INFO] Test STOP\n\n";
  }
};

class LocalPlannerTestClass : public LocalPlanner {

public:
  LocalPlannerTestClass() = default;
  ~LocalPlannerTestClass() = default;

  void runTest() {

    std::cout << "[INFO] Running 'drone_local_planner_test'\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new goal'\n";

    Eigen::Vector3f goal = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
    this->setGoal(goal);

    if (this->goal_updated_) {
      if (eqmarg(this->goal_.x(), 1.0f, 0.0f) && 
          eqmarg(this->goal_.y(), 1.0f, 0.0f) && 
          eqmarg(this->goal_.z(), 1.0f, 0.0f))
        std::cout << "[SUCCESS] New goal updated\n";
      else
        std::cout << "[ERROR] Mismatch on new goal: (" << this->goal_.x() << ", " << this->goal_.y() << ", " << this->goal_.z() 
                  << ") =/= (1.0, 1.0, 1.0)\n";
      
      if (eqmarg(this->goal_pos_.x(), 0.387, 0.001f) &&
          eqmarg(this->goal_pos_.y(), 0.387, 0.001f) &&
          eqmarg(this->goal_pos_.z(), 1.5f, 0.001f))
        std::cout << "[SUCCESS] New goal position updated\n";
      else
        std::cout << "[ERROR] Mismatch on new goal position: (" << this->goal_pos_.x() << ", " << this->goal_pos_.y() << ", " 
                  << this->goal_pos_.z() << ") =/= (0.387.., 0.387.., 1.5)\n";
    }
    else
      std::cout << "[ERROR] Could not add new goal\n";
    
    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new goal within ignorance margin'\n";

    goal = Eigen::Vector3f(1.01, 1.01, 1.01);
    this->setGoal(goal);

    if (this->goal_updated_) {
      std::cout << "[ERROR] New goal updated despite being inside ignorance margin\n";
    }
    else
      std::cout << "[SUCCESS] New goal ingored (point within ignorance margin)\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new goal with preserving previuos goal\n";

    goal = Eigen::Vector3f(2.0, 2.0, 2.0);
    this->setGoal(goal);

    if (this->goal_updated_) {
      Eigen::Vector3f prev_goal = this->prev_goal_array_[1];
      if (eqmarg(this->goal_.x(), 2.0f, 0.0f) && 
          eqmarg(this->goal_.y(), 2.0f, 0.0f) && 
          eqmarg(this->goal_.z(), 2.0f, 0.0f))
        std::cout << "[SUCCESS] New goal updated\n";
      else
        std::cout << "[ERROR] Mismatch on new goal: (" << this->goal_.x() << ", " << this->goal_.y() << ", " << this->goal_.z() 
                  << ") =/= (2.0, 2.0, 2.0)\n";
      if (eqmarg(prev_goal.x(), 1.0f, 0.0f) && 
          eqmarg(prev_goal.y(), 1.0f, 0.0f) && 
          eqmarg(prev_goal.z(), 1.0f, 0.0f))
        std::cout << "[SUCCESS] Previous goal preserved\n";
      else
        std::cout << "[ERROR] Mismatch on previous goal: ("<< prev_goal.x() << ", " << prev_goal.y() << ", " << prev_goal.z() 
                  << ") =/= (1.0, 1.0, 1.0)\n";
    }
    else
      std::cout << "[ERROR] Could not add new goal\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'update state'\n";

    Eigen::Vector3f position    = Eigen::Vector3f::Zero();
    Eigen::Vector3f orientation = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity    = Eigen::Vector3f::Zero();

    this->state_updated_ = false;

    this->setState(position, orientation, velocity);

    bool no_mismatch = true;

    if (this->state_updated_) {
      if (!eqmarg(this->position_.x(), 0.0f, 0.0f) ||
          !eqmarg(this->position_.y(), 0.0f, 0.0f) ||
          !eqmarg(this->position_.z(), 0.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: position\n";
      }
      if (!eqmarg(this->orientation_.x(), 0.0f, 0.0f) ||
          !eqmarg(this->orientation_.y(), 0.0f, 0.0f) ||
          !eqmarg(this->orientation_.z(), 0.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: orientation\n";
      }
      if (!eqmarg(this->lin_velocity_.x(), 0.0f, 0.0f) ||
          !eqmarg(this->lin_velocity_.y(), 0.0f, 0.0f) ||
          !eqmarg(this->lin_velocity_.z(), 0.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: velocity\n";
      }
      if (!eqmarg(this->fov_.yaw_deg, 0.0f, 0.0f) || 
          !eqmarg(this->fov_.pitch_deg, 0.0f, 0.0f) || 
          !eqmarg(this->fov_.h_fov_deg, this->config_.camera_fov_h, 0.0f) || 
            !eqmarg(this->fov_.v_fov_deg, this->config_.camera_fov_v, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: FOV\n";
      }
      if (no_mismatch)
        std::cout << "[SUCCESS] State updated\n";
    }
    else
      std::cout << "[ERROR] Could not update state\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'update state - non-zero input'\n";

    position    = Eigen::Vector3f(1.0f, 2.0f, 3.0f);
    orientation = Eigen::Vector3f(0.5f, 1.0f, 1.5f);
    velocity    = Eigen::Vector3f(0.5f, 0.5f, 0.5f);

    this->state_updated_ = false;

    this->setState(position, orientation, velocity);

    no_mismatch = true;

    if (this->state_updated_) {
      if (!eqmarg(this->position_.x(), 1.0f, 0.0f) ||
          !eqmarg(this->position_.y(), 2.0f, 0.0f) ||
          !eqmarg(this->position_.z(), 3.0f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: position\n";
      }
      if (!eqmarg(this->orientation_.x(), 0.5f, 0.0f) ||
          !eqmarg(this->orientation_.y(), 1.0f, 0.0f) ||
          !eqmarg(this->orientation_.z(), 1.5f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: orientation\n";
      }
      if (!eqmarg(this->lin_velocity_.x(), 0.5f, 0.0f) ||
          !eqmarg(this->lin_velocity_.y(), 0.5f, 0.0f) ||
          !eqmarg(this->lin_velocity_.z(), 0.5f, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: velocity\n";
      }
      if (!eqmarg(this->fov_.yaw_deg, 1.5f * RAD_TO_DEG, 0.001f) || 
          !eqmarg(this->fov_.pitch_deg, 1.0f * RAD_TO_DEG, 0.001f) || 
          !eqmarg(this->fov_.h_fov_deg, this->config_.camera_fov_h, 0.0f) || 
          !eqmarg(this->fov_.v_fov_deg, this->config_.camera_fov_v, 0.0f)) {
        no_mismatch = false;
        std::cout << "[ERROR] Mismatch on updated state: FOV\n";
      }
      if (no_mismatch)
        std::cout << "[SUCCESS] State updated\n";
    }
    else
      std::cout << "[ERROR] Could not update state\n";
    
    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new point cloud\n";

    position    = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    this->setState(position, orientation, velocity);

    PointXYZ point_xyz = PointXYZ(1.0, 1.0, 1.0);
    PointCloud<PointXYZ> point_cloud = PointCloud<PointXYZ>();
    point_cloud.push_back(point_xyz);

    this->base_cloud_.clear();
    this->setPointCloud(point_cloud);

    if (this->cloud_updated_) {
      if (this->cloud_cache_.size() != 1)
        std::cout << "[ERROR] Incorrect point cloud size: '" << this->cloud_cache_.size() << "' =/= 1\n";
      else if(!eqmarg(this->cloud_cache_[0].x, 2.0f, 0.0f) || 
              !eqmarg(this->cloud_cache_[0].y, 2.0f, 0.0f) || 
              !eqmarg(this->cloud_cache_[0].z, 1.0f, 0.0f))
        std::cout << "[ERROR] Mismatch on 3D point in added point cloud: (" << this->cloud_cache_[0].x << ", " 
                  << this->cloud_cache_[0].y << ", " << this->cloud_cache_[0].z << ") =/= (2.0, 2.0, 1.0)\n";
      else
        std::cout << "[SUCCESS] New point cloud updated\n";
    }
    else
      std::cout << "[ERROR] Could not add new point cloud\n";
    
    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: 'add new point cloud - non-zero orientation\n";

    position    = Eigen::Vector3f(1.0f, 1.0f, 0.0f);
    orientation = Eigen::Vector3f(0.0f, -PI_F / 4.0f, PI_F / 4.0f);
    velocity    = Eigen::Vector3f::Zero();

    this->setState(position, orientation, velocity);

    point_xyz = PointXYZ(1.0, 0.0, 0.0);
    point_cloud.clear();
    point_cloud.push_back(point_xyz);

    this->base_cloud_.clear();
    this->setPointCloud(point_cloud);

    if (this->cloud_updated_) {
      if (this->cloud_cache_.size() != 1)
        std::cout << "[ERROR] Incorrect point cloud size: '" << this->cloud_cache_.size() << "' =/= 1\n";
      else if(!eqmarg(this->cloud_cache_[0].x, 1.5f, 0.001f) || 
              !eqmarg(this->cloud_cache_[0].y, 1.5f, 0.001f) || 
              !eqmarg(this->cloud_cache_[0].z, 0.707f, 0.001f))
        std::cout << "[ERROR] Mismatch on 3D point in added point cloud: (" << this->cloud_cache_[0].x << ", " 
                  << this->cloud_cache_[0].y << ", " << this->cloud_cache_[0].z << ") =/= (1.5, 1.5, 0.707..)\n";
      else
        std::cout << "[SUCCESS] New point cloud updated\n";
    }
    else
      std::cout << "[ERROR] Could not add new point cloud\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: processing point cloud\n";

    position    = Eigen::Vector3f::Zero();
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    this->setState(position, orientation, velocity);

    this->fov_.h_fov_deg = 90.0f;
    this->fov_.v_fov_deg = 90.0f;

    this->base_cloud_.clear();
    this->base_cloud_.push_back(PointXYZI(1.0, 0.0, 0.0, 0.0));  // inside FOV - will be overwritten
    this->base_cloud_.push_back(PointXYZI(0.5, 0.0, 1.0, 0.0));  // outside FOV - will be preserved
    this->base_cloud_.push_back(PointXYZI(0.5, 0.0, 1.0, 10.0)); // too old - will be discarded
    
    point_cloud.clear();
    point_cloud.push_back(PointXYZ(0.5, 0.0, 0.5)); // inside FOV - will be saved
    point_cloud.push_back(PointXYZ(0.5, 0.0, 0.6)); // outside FOV - will be discarded
    point_cloud.push_back(PointXYZ(15.0, 0.0, 0.0)); // outside sensor range - will be discarded
    this->setPointCloud(point_cloud);

    this->processPointCloud();

    if (this->base_cloud_.size() == 2) {
      if (eqmarg(this->base_cloud_[0].x, 0.5f, 0.0f) && 
          eqmarg(this->base_cloud_[0].z, 0.5f, 0.0f) && 
          eqmarg(this->base_cloud_[0].i, 0.0f, 0.0f) && 
          eqmarg(this->base_cloud_[1].x, 0.5f, 0.0f) && 
          eqmarg(this->base_cloud_[1].z, 1.0f, 0.0f) && 
          this->base_cloud_[1].i > 0.0f)
        std::cout << "[SUCCESS] Point cloud processed w/o issues\n";
      else
        std::cout << "[ERROR] Point cloud contatins unexpected points\n";
    }
    else
      std::cout << "[ERROR] Incorrect point cloud size: '" << this->base_cloud_.size() << "' =/= 2\n";
    std::cout << "Print cloud:\n";
    for (const PointXYZI& point : this->base_cloud_) std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ") (" << point.i << ")\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: empty run\n";

    this->base_cloud_.clear();
    this->prev_goal_array_.clear();
    this->next_ = Eigen::Vector3f::Zero();
    this->rotation_matrix_ = Eigen::AngleAxisf(this->orientation_.z(), Eigen::Vector3f::UnitZ()) *
                             Eigen::AngleAxisf(this->orientation_.y(), Eigen::Vector3f::UnitY()) *
                             Eigen::AngleAxisf(this->orientation_.x(), Eigen::Vector3f::UnitX());

    position    = Eigen::Vector3f::Zero();
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();
    
    this->setState(position, orientation, velocity);

    goal = Eigen::Vector3f(1.7321f, 0.0f, 0.0f);

    this->setGoal(goal);

    point_cloud.clear();

    this->setPointCloud(point_cloud);

    this->run();

    Eigen::Vector3f next_pos = this->getNext();

    // due to character of polar histogram (even resolution) candidate direction cannot point straight forward
    if (eqmarg(next_pos.x(), 0.889f, 0.001f) && 
        eqmarg(next_pos.y(), 0.046f, 0.001f) && 
        eqmarg(next_pos.z(), 0.453f, 0.001f))
      std::cout << "[SUCCESS] Empty run w/o issues\n";
    else
      std::cout << "[ERROR] Incorrect next goal drone pos: (" << next_pos.x() << ", " << next_pos.y() << ", " << next_pos.z() 
                << ") =/= (0.889.., 0.046.., 0.453)\n";
    std::cout << "Print target position: (" << this->goal_.x() << ", " << this->goal_.y() << ", " << this->goal_.z() << ")\n";
    std::cout << "Print goal position: (" << this->goal_pos_.x() << ", " << this->goal_pos_.y() << ", " << this->goal_pos_.z() 
              << "), distance: (" << this->goal_pos_.norm() << ")\n";
    
    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: timing adding point cloud\n";

    position    = Eigen::Vector3f::Zero();
    orientation = Eigen::Vector3f::Zero();
    velocity    = Eigen::Vector3f::Zero();

    goal = Eigen::Vector3f(1.0f, 0.0f, 1.0f);

    point_cloud.clear();

    int c_width = 640;
    int c_height = 360;

    double step_w = 10.0 / (c_width / 2);
    double step_h = 5.0 / (c_height / 2);
    double step_d = 1.0 / (c_width / 2);

    Eigen::Vector3f sp = Eigen::Vector3f(step_d, step_w, 0.0);
    for (int i = 0; i < c_height; i++) {
      Eigen::Vector3f np = Eigen::Vector3f(5.0, -10.0, -5.0);
      for (int j = 0; j < c_width; j++) {
        point_cloud.push_back(PointXYZ(np.x(), np.y(), np.z()));
        np += sp;
      }
      np += Eigen::Vector3f(0.0, 0.0, step_h);
    }

    this->reset();

    double avg_time = 0.0;
    double max_time = 0.0;
    double min_time = 1e3;
    double passed = 0.0;
    std::chrono::duration<double> time_passed;
    int n_loops = 100;

    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->setPointCloud(point_cloud);

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: timing processing point cloud\n";

    this->base_cloud_.clear();
    this->setState(position, orientation, velocity);
    this->setPointCloud(point_cloud);
    this->processPointCloud();

    avg_time = 0.0;
    max_time = 0.0;
    min_time = 1e3;
    passed = 0.0;
    n_loops = 100;

    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      this->histogram_.clear();
      this->cloud_updated_ = true;

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->processPointCloud();

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    // ******************************************************************************************************************************** //
    std::cout << "[INFO] Test: timing plannig route\n";

    avg_time = 0.0;
    max_time = 0.0;
    min_time = 1e3;
    passed = 0.0;
    n_loops = 100;

    std::cout << "Running " << n_loops << " loops\n";
    for (int l = 0; l < n_loops; l++) {

      this->histogram_.clear();
      this->cloud_updated_ = true;

      std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

      this->planNext();

      time_passed = std::chrono::system_clock::now() - time_start;
      passed = time_passed.count();
      avg_time += passed;
      max_time = passed > max_time ? passed : max_time;
      min_time = passed < min_time ? passed : min_time;
    }
    avg_time = avg_time / n_loops;
    std::cout << "Processing time: average: " << avg_time << "; max: " << max_time << "; min: " << min_time << " [s]\n";

    std::cout << "[INFO] Test STOP\n\n";
  }
};

} // namespace DRONE_NAVIGATION


int main() {

  DRONE_NAVIGATION::PolarHistogramTestClass polar_histogram_test_class = DRONE_NAVIGATION::PolarHistogramTestClass();
  polar_histogram_test_class.runTest();

  DRONE_NAVIGATION::LocalPlannerTestClass local_planner_test_class = DRONE_NAVIGATION::LocalPlannerTestClass();
  local_planner_test_class.runTest();

  return 0;
}