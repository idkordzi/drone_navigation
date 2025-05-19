#pragma once

#include <chrono>
#include "Eigen/Dense"


namespace DRONE_NAVIGATION {

struct StateEstimatorConfig {
  const unsigned state_num_ = 1;
  const unsigned measurement_num_ = 1;
};

class StateEstimator {

public:

  StateEstimator();
  StateEstimator(StateEstimatorConfig config);
  ~StateEstimator() = default;

  void runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement);
  void runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement, float dt);

  Eigen::VectorXf getState() const {return this->xe_;}

protected:

  void initialize();

  StateEstimatorConfig config_ = {};

  std::chrono::system_clock::time_point ts_ = {};

  Eigen::VectorXf xe_ = {}; // states estimates
  Eigen::MatrixXf Pe_ = {}; // estimations covariance

  Eigen::MatrixXf F_ = {}; // model Jacobian
  Eigen::MatrixXf H_ = {}; // measurements Jacobian

  Eigen::MatrixXf Q_ = {}; // model covariance matrix
  Eigen::MatrixXf R_ = {}; // measurements covariance matrix
};

} // namespace DRONE_NAVIGATION
