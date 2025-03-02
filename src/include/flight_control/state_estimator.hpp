#pragma once


namespace DRONE_NAVIGATION {

struct StateEstimatorConfig {};

class StateEstimator {

public:

  StateEstimator();
  ~StateEstimator() = default;

private:

  StateEstimatorConfig config_ = {};
};

} // namespace DRONE_NAVIGATION
