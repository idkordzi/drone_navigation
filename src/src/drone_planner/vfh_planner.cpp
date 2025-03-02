#include "vfh_planner.hpp"


namespace DRONE_NAVIGATION {

VFHPlanner::VFHPlanner() {
  this->planner_state_ = 1;
}

void VFHPlanner::updateStatus() {
  if (this->goal_updated_)     this->planner_state_ = 1;
  else if (!this->checkNode()) this->planner_state_ = 1;
  else                         this->planner_state_ = 2;
}

void VFHPlanner::run() {

  // Drone far away from final goal
  if ((this->goal_-this->position_).norm() > this->vhf_planner_config_.drone_pos_margin_) {
    
    // Goal changed / no trajectory available
    if (this->planner_state_ == 1) {
      FOV new_fov(this->orientation_.z(), this->orientation_.y(), this->local_planner_config_.camera_fov_h, this->local_planner_config_.camera_fov_v);
      this->setFOV(new_fov);

      this->processPointCloud();
      PolarHistogram histogram(CELL_SIZE);
      this->generateHistogram(histogram, this->position_);
      this->searchForPath();

      // TODO handle case of no available node on the path
      if(this->getNode(this->next_)) this->dropNode();
    }

    // Drone has not reached closest node on the path
    if ((this->next_-this->position_).norm() > this->vhf_planner_config_.drone_pos_margin_) {
      // do nothing, uphold control
    }
    // Drone reached path node
    else {
      // TODO handle case of no available node on the path
      if(this->getNode(this->next_)) this->dropNode();
    }
  }
  // Drone within acceptable margin
  else {
    // TODO handle case when drone is at the target location
  }
}

} // namespace DRONE_NAVIGATION
