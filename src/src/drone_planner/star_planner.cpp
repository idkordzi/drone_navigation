#include "star_planner.hpp"


namespace DRONE_NAVIGATION {

TreeNode::TreeNode() : total_cost_{0.0f}, heuristic_{0.0f}, origin_{0}, depth_(0), closed_{false} {
  this->position_ = Eigen::Vector3f::Zero();
  this->velocity_ = Eigen::Vector3f::Zero();
}

TreeNode::TreeNode(int from, const Eigen::Vector3f& pos, const Eigen::Vector3f& vel)
: total_cost_{0.0f}, heuristic_{0.0f}, origin_{from}, depth_(0), closed_{false}
{
  this->position_ = pos;
  this->velocity_ = vel;
}


void StarPlanner::searchForPath() {

  PolarHistogram histogram(CELL_SIZE);
  Eigen::MatrixXf cost_matrix;
  std::vector<MoveDirection> direction_list;

  bool is_expanded = true;

  this->tree_.clear();

  this->tree_.push_back(TreeNode(0, this->position_, this->lin_velocity_));
  this->tree_.back().total_cost_ = this->heuristicFunction(0);
  this->tree_.back().heuristic_ = this->heuristicFunction(0);

  int origin = 0;
  for (int ni = 0; ni < this->star_planner_config_.max_expanded_nodes_ && is_expanded; ni++) {
    Eigen::Vector3f origin_position = this->tree_[origin].position_;
    Eigen::Vector3f origin_velocity = this->tree_[origin].velocity_;

    histogram.clear();
    this->generateHistogram(histogram, origin_position);

    // calculate candidates
    cost_matrix.fill(0.0f);
    direction_list.clear();
    this->getCostMatrix(histogram, origin_position, origin_velocity, cost_matrix); // origin_position, origin_velocity
    this->getBestMoveDirections(cost_matrix, direction_list);

    // add candidates as nodes
    if (direction_list.empty()) {
      this->tree_[origin].total_cost_ = HUGE_VALF;
      this->tree_[origin].heuristic_  = HUGE_VALF;
    }
    else {
      // insert new nodes
      int children = 0;
      for (MoveDirection direction : direction_list) {
        PolarPoint polar = PolarPoint(direction.elevation_, direction.azimuth_, this->star_planner_config_.node_step_distance_);
        Eigen::Vector3f child_posision = convertPolarToCartesian(polar, origin_position);
        Eigen::Vector3f child_velocity = child_posision - origin_position;  // @TODO: verify & improve

        // check if another close node has been added
        int nearby_nodes_cnt = 0;
        for (size_t i = 0; i < tree_.size(); i++) {
          float distance = (tree_[i].position_ - child_posision).norm();
          if (distance < this->star_planner_config_.node_distance_margin_) {
            nearby_nodes_cnt++;
            break;
          }
        }

        if (children < this->star_planner_config_.children_per_node_ && nearby_nodes_cnt == 0) {
          this->tree_.push_back(TreeNode(origin, child_posision, child_velocity));
          float hc = this->heuristicFunction(this->tree_.size() - 1);
          this->tree_.back().total_cost_ = tree_[origin].total_cost_ - tree_[origin].heuristic_ + direction.cost_ + hc;
          this->tree_.back().heuristic_ = hc;
          this->tree_.back().depth_ = tree_[origin].depth_ + 1;
          children++;
        }
      }
    }

    tree_[origin].closed_ = true;

    // find best node to continue
    float minimal_cost = HUGE_VAL;
    is_expanded = false;
    for (size_t i = 0; i < tree_.size(); i++) {
      if (!(tree_[i].closed_)) {
        float node_distance = (tree_[i].position_ - this->position_).norm();
        if (tree_[i].total_cost_ < minimal_cost && node_distance < this->star_planner_config_.max_path_length_) {
          minimal_cost = tree_[i].total_cost_;
          origin = i;
          is_expanded = true;
        }
      }
    }

    direction_list.clear();
  }

  // find best node to follow, taking into account A* completion
  int max_depth = 0;
  int max_depth_index = 0;
  for (size_t i = 0; i < tree_.size(); i++) {
    if (!(tree_[i].closed_)) {
      if (tree_[i].depth_ > max_depth) {
        max_depth = tree_[i].depth_;
        max_depth_index = i;
      }
    }
  }

  // build final tree
  int tree_end = max_depth_index;
  this->base_path_.clear();
  while (tree_end > 0) {
    this->base_path_.push_back(tree_[tree_end].position_);
    tree_end = tree_[tree_end].origin_;
  }
  this->base_path_.push_back(tree_[0].position_);

}

bool StarPlanner::dropNode() {
  if (this->base_path_.size() > 0) {
    this->base_path_.pop_back();
    return true;
  } else {
    return false;
  }
}

bool StarPlanner::getNode(Eigen::Vector3f& node) const {
  if (this->base_path_.size() > 0) {
    node = this->base_path_.back();
    return true;
  } else {
    node.fill(NAN);
    return false;
  }
}

bool StarPlanner::checkNode() const {
  return this->base_path_.size() > 0;
}

CostFunctionOutput StarPlanner::costFunction(const PolarPoint& candidate, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, 
                                             float obstacle_distance, bool is_obstacle_facing_goal) const
{
  const PolarPoint facing_goal = convertCartesianToPolar(this->goal_, position);
  const float goal_distance = (this->goal_ - position).norm();
  const Eigen::Vector3f candidate_velocity_cartesian = convertPolarToCartesian(candidate, Eigen::Vector3f(0.0f, 0.0f, 0.0f));

  const float angle_diff = angleDifference(candidate.azim, facing_goal.azim);

  const PolarPoint facing_line = convertCartesianToPolar(this->next_, position);
  const float angle_diff_to_line = angleDifference(candidate.azim, facing_line.azim);

  const float velocity_cost = star_planner_config_.velocity_cost_param * (velocity.norm() - 
                              candidate_velocity_cartesian.normalized().dot(velocity));

  float weight = 0.f;  // Yaw cost partition between back to line previous-current goal and goal
  if (!is_obstacle_facing_goal) weight = 0.5f;

  const float yaw_cost         = (1.f - weight) * star_planner_config_.yaw_cost_param * angle_diff * angle_diff;
  const float yaw_to_line_cost = weight * star_planner_config_.yaw_cost_param * angle_diff_to_line * angle_diff_to_line;
  float pitch_cost             = star_planner_config_.pitch_cost_param * (candidate.elev - facing_goal.elev) * (candidate.elev - facing_goal.elev);
  
  //Increase the pitch cost starting at 5m from the goal (forcing the drone to goal altitude)
  if (goal_distance < 5.f) pitch_cost = pitch_cost / ((0.2 * goal_distance) * (0.2 * goal_distance));

  const float d = star_planner_config_.obstacle_cost_param - obstacle_distance;
  const float distance_cost = obstacle_distance > 0 ? 5000.0f * (1 + d / sqrt(1 + d * d)) : 0.0f;

  return CostFunctionOutput(distance_cost, velocity_cost + yaw_cost + yaw_to_line_cost + pitch_cost);

}

float StarPlanner::heuristicFunction(int node_idx) const {
  return (this->goal_ - this->tree_[node_idx].position_).norm() * this->star_planner_config_.heuristic_weight_;
}

void StarPlanner::getCostMatrix(const PolarHistogram& histogram, const Eigen::Vector3f& position,
                                const Eigen::Vector3f& velocity, Eigen::MatrixXf& cost_matrix) const
{
  Eigen::MatrixXf distance_matrix(ELEVATION_RESOLUTION, AZIMUTH_RESOLUTION);
  distance_matrix.fill(NAN);

  cost_matrix.resize(ELEVATION_RESOLUTION, AZIMUTH_RESOLUTION);
  cost_matrix.fill(NAN);

  // Look if there are any obstacles in the goal direcion +/-33deg azimuth, +/-15deg elevation
  PolarPoint goal_polar        = convertCartesianToPolar(this->goal_, position);
  Eigen::Vector2i goal_index   = convertPolarToHistogramIndex(goal_polar, CELL_SIZE);
  bool is_obstacle_facing_goal = false;

  int elev_qnt = 30 / CELL_SIZE / 2; // elevation 5*ALPHA_RES = 30deg
  int azim_qnt = 66 / CELL_SIZE / 2; // azimuth  11*ALPHA_RES = 66deg
  for (int j = -elev_qnt; j <= elev_qnt; j++) {
    for (int i = -azim_qnt; i <= azim_qnt; i++) {
      PolarPoint polar(goal_polar.elev, goal_polar.azim, 0.0);
      polar.elev += (float)j * CELL_SIZE;
      polar.azim += (float)i * CELL_SIZE;
      Eigen::Vector2i ind = convertPolarToHistogramIndex(polar, CELL_SIZE);
      if (histogram.getDistance(ind.y(), ind.x()) > this->local_planner_config_.sensor_min_range_ &&
          histogram.getDistance(ind.y(), ind.x()) < this->local_planner_config_.sensor_max_range_)
        is_obstacle_facing_goal = (is_obstacle_facing_goal || true);
    }
  }

  // Fill in cost matrix
  for (int elev = 0; elev < ELEVATION_RESOLUTION; elev++) {
    // Determine how many bins at this elevation angle would be equivalent 
    // to a single bin at horizontal, then work in steps of that size
    const float bin_width = std::cos(convertHistogramIndexToPolar(elev, 0, CELL_SIZE, 1).elev * DEG_TO_RAD);
    const int step_size = static_cast<int>(std::round(1 / bin_width));

    for (int azim = 0; azim < AZIMUTH_RESOLUTION; azim += step_size) {
      float obstacle_distance = histogram.getDistance(elev, azim);
      PolarPoint polar = convertHistogramIndexToPolar(elev, azim, CELL_SIZE, 1.0f);  // unit vector of current direction
      CostFunctionOutput costs = costFunction(polar, position, velocity, obstacle_distance, is_obstacle_facing_goal);
      cost_matrix(elev, azim) = costs.distance_cost_;
      distance_matrix(elev, azim) = costs.state_cost_;
    }
    if (step_size > 1) {

      // Horizontally interpolate all of the un-calculated values
      int last_index = 0;
      for (int azim = step_size; azim < AZIMUTH_RESOLUTION; azim += step_size) {
        float other_costs_gradient   = (cost_matrix(elev, azim) - cost_matrix(elev, last_index)) / step_size;
        float distance_cost_gradient = (distance_matrix(elev, azim) - distance_matrix(elev, last_index)) / step_size;
        for (int i = 1; i < step_size; i++) {
          cost_matrix(elev, last_index + i)     = cost_matrix(elev, last_index) + other_costs_gradient * i;
          distance_matrix(elev, last_index + i) = distance_matrix(elev, last_index) + distance_cost_gradient * i;
        }
        last_index = azim;
      }

      // Special case the last columns wrapping around back to 0
      int clamped_z_scale = AZIMUTH_RESOLUTION - last_index;
      float other_costs_gradient   = (cost_matrix(elev, 0) - cost_matrix(elev, last_index)) / clamped_z_scale;
      float distance_cost_gradient = (distance_matrix(elev, 0) - distance_matrix(elev, last_index)) / clamped_z_scale;
      for (int i = 1; i < clamped_z_scale; i++) {
        cost_matrix(elev, last_index + i)     = cost_matrix(elev, last_index) + other_costs_gradient * i;
        distance_matrix(elev, last_index + i) = distance_matrix(elev, last_index) + distance_cost_gradient * i;
      }
    }
  }

  cost_matrix = cost_matrix + distance_matrix;
}

void StarPlanner::getBestMoveDirections(const Eigen::MatrixXf& cost_matrix, std::vector<MoveDirection>& direction_list) const {

  direction_list.clear();
  for (int i = 0; i < cost_matrix.rows(); i++) {
    for (int j = 0; j < cost_matrix.cols(); j++) {
      PolarPoint polar = convertHistogramIndexToPolar(i, j, CELL_SIZE, 1.0);
      MoveDirection candidate(polar.elev, polar.azim, cost_matrix(i, j));

      unsigned it = 0;
      while (it < direction_list.size() && candidate > direction_list[it]) it++;
      direction_list.insert(std::next(direction_list.begin(), it), candidate);
      if (direction_list.size() > this->star_planner_config_.children_per_node_) direction_list.pop_back();
    }
  }
}

} // namespace DRONE_NAVIGATION
