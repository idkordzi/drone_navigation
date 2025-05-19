#include "state_estimator.hpp"


namespace DRONE_NAVIGATION {

StateEstimator::StateEstimator() {
  this->initialize();
}

StateEstimator::StateEstimator(StateEstimatorConfig config) : config_(config) {
  this->initialize();
}

void StateEstimator::initialize() {
  // @TODO
}

void StateEstimator::runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement) {
  std::chrono::duration<double> time_passed = std::chrono::system_clock::now() - this->ts_;
  this->runEstimate(input, measurement, time_passed.count());
  this->ts_ = std::chrono::system_clock::now();
}

void StateEstimator::runEstimate(Eigen::VectorXf input, Eigen::VectorXf measurement, float dt) {
  
  // prep
  Eigen::MatrixXf I = {};

  // prediction step
  Eigen::VectorXf xp = {};
  Eigen::MatrixXf Pp = this->F_ * this->Pe_ * this->F_.transpose() + this->Q_;

  // update step
  Eigen::VectorXf y = {};
  Eigen::MatrixXf S = this->H_ * Pp * this->H_.transpose() + this->R_;
  Eigen::MatrixXf K = Pp * this->H_ * S.inverse();

  // update output
  this->xe_ = xp + K * y;
  this->Pe_ = (I -K * this->H_) * Pp;
}

} // namespace DRONE_NAVIGATION
