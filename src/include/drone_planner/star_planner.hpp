#pragma once

#include "Eigen/Dense"
#include "local_planner.hpp"


namespace DRONE_NAVIGATION {

class TreeNode {

public:
  TreeNode();
  TreeNode(int from, const Eigen::Vector3f& pos, const Eigen::Vector3f& vel);
  ~TreeNode() = default;

  Eigen::Vector3f position_ = {};
  Eigen::Vector3f velocity_ = {};

  float total_cost_ = 0.0f;
  float heuristic_  = 0.0f;
  int origin_  =  0;
  int depth_   = 0;
  bool closed_ = false;
};


struct MoveDirection {

  MoveDirection(float elev, float azim, float cost) : elevation_(elev), azimuth_(azim), cost_(cost) {}

  bool operator<(const MoveDirection& d) const { return this->cost_ < d.cost_; }
  bool operator>(const MoveDirection& d) const { return this->cost_ > d.cost_; }
  
  float elevation_ = 0.0f;
  float azimuth_   = 0.0f;
  float cost_      = 0.0f;
};


struct CostFunctionOutput {

  CostFunctionOutput() : distance_cost_(0.0f), state_cost_(0.0f) {}
  CostFunctionOutput(float d, float s) : distance_cost_(d), state_cost_(s) {}

  float distance_cost_ = 0.0f;
  float state_cost_    = 0.0f;
};


class StarPlanner : public LocalPlanner {

public:

  StarPlanner() {};
  ~StarPlanner() = default;

  std::vector<Eigen::Vector3f> getPath() const;

protected:

  void searchForPath();
  bool dropNode();
  bool getNode(Eigen::Vector3f& node) const;
  bool checkNode() const;

  CostFunctionOutput costFunction(const PolarPoint& candidate, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, 
                                  float obstacle_distance, bool is_obstacle_facing_goal) const;
  float heuristicFunction(int node_idx) const;

  void getCostMatrix(const PolarHistogram& histogram, const Eigen::Vector3f& position,
                     const Eigen::Vector3f& velocity, Eigen::MatrixXf& cost_matrix) const;
  void getBestMoveDirections(const Eigen::MatrixXf& cost_matrix, std::vector<MoveDirection>& direction_list) const;

  StarPlannerConfig star_planner_config_ = {};

  std::vector<Eigen::Vector3f> base_path_ = {};
  std::vector<TreeNode> tree_ = {};

};

} // namespace DRONE_NAVIGATION
