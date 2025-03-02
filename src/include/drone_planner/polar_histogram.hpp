#pragma once

#include <cmath>
#include <float.h>
#include <vector>
#include <stdexcept>
#include "Eigen/Dense"


namespace DRONE_NAVIGATION {

// Valid resolutions must fullfill:
// 180 % (2 * ALPHA_RES) = 0
// Examples: 1, 3, 5, 6, 10, 15, 18, 30, 45, 60
constexpr int CELL_SIZE = 6;
constexpr int AZIMUTH_RESOLUTION   = 360 / CELL_SIZE;
constexpr int ELEVATION_RESOLUTION = 180 / CELL_SIZE;

class PolarHistogram {

public:
  PolarHistogram(int res);
  ~PolarHistogram() = default;

  float getDistance(int elev, int azim) const {
    wrapIndex(elev, azim);
    return this->distance_(elev, azim);
  }

  void setDistance(int elev, int azim, float value) {
    this->distance_(elev, azim) = value;
  }

  void upsample();
  void downsample();
  void clear();
  bool isEmpty() const;

private:

  int resolution_ = 0.0f;
  int azim_dim_   = 0.0f;
  int elev_dim_   = 0.0f;
  
  Eigen::MatrixXf distance_ = {};

  inline void wrapIndex(int &elev, int &azim) const {
    elev = elev % this->elev_dim_;
    if (elev < 0) elev += this->elev_dim_;
    azim = azim % this->azim_dim_;
    if (azim < 0) azim += this->azim_dim_;
  }

};

} // namespace DRONE_NAVIGATION
