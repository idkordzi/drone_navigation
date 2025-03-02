#include "common.hpp"


namespace DRONE_NAVIGATION {

bool pointInsideFOV(const FOV& fov, const PolarPoint& polar) {
  return polar.azim <= wrapRangeDeg(fov.yaw_deg + fov.h_fov_deg / 2.f) &&
         polar.azim >= wrapRangeDeg(fov.yaw_deg - fov.h_fov_deg / 2.f) &&
         polar.elev <= fov.pitch_deg + fov.v_fov_deg / 2.f &&
         polar.elev >= fov.pitch_deg - fov.v_fov_deg / 2.f;
}
    
PolarPoint convertCartesianToPolar(const Eigen::Vector3f& point, const Eigen::Vector3f& origin) {
  PolarPoint polar(0.0f, 0.0f, 0.0f);
  float den = (Eigen::Vector2f(point.x(), point.y()) - origin.topRows<2>()).norm();
  polar.elev = std::atan2(point.z() - origin.z(), den) * RAD_TO_DEG;                     // ( -90,  +90)
  polar.azim = std::atan2(point.x() - origin.x(), point.y() - origin.y()) * RAD_TO_DEG;  // (-180, +180]
  polar.radi = sqrt((point.x() - origin.x()) * (point.x() - origin.x()) + 
                    (point.y() - origin.y()) * (point.y() - origin.y()) + 
                    (point.z() - origin.z()) * (point.z() - origin.z()));
  return polar;
}
    
Eigen::Vector3f convertPolarToCartesian(const PolarPoint& polar, const Eigen::Vector3f& origin) {
  Eigen::Vector3f point;
  point.x() = origin.x() + polar.radi * std::cos(polar.elev * DEG_TO_RAD) * std::sin(polar.azim * DEG_TO_RAD);
  point.y() = origin.y() + polar.radi * std::cos(polar.elev * DEG_TO_RAD) * std::cos(polar.azim * DEG_TO_RAD);
  point.z() = origin.z() + polar.radi * std::sin(polar.elev * DEG_TO_RAD);
  return point;
}
    
void wrapPolar(PolarPoint& polar) {
  polar.elev = wrapRangeDeg(polar.elev);
  polar.azim = wrapRangeDeg(polar.azim);

  bool wrapped = false;
  if (polar.elev > 90.0f) {
    polar.elev = 180.0f - polar.elev;
    wrapped = true;
  } else if (polar.elev < -90.0f) {
    polar.elev = -(180.0f + polar.elev);
    wrapped = true;
  }
  if (wrapped) {
    if (polar.azim < 0.0f) polar.azim += 180.0f;
    else polar.azim -= 180.0f;
  }
}
    
Eigen::Vector2i convertPolarToHistogramIndex(const PolarPoint& polar, int res) {
  Eigen::Vector2i idx(0, 0);
  PolarPoint wrapped = polar;
  wrapPolar(wrapped);
  
  idx.y() = (int)(std::floor((wrapped.elev + 90.0f) / res));
  idx.x() = (int)(std::floor((wrapped.azim +180.0f) / res));
  
  if (idx.x() >= 360 / res) idx.x() = 360 / res - 1;
  if (idx.x() < 0) idx.x() = 0;
  if (idx.y() >= 180 / res) idx.y() = 180 / res - 1;
  if (idx.y() < 0) idx.y() = 0;
  
  return idx;
}
    
PolarPoint convertHistogramIndexToPolar(int elev, int azim, int res, float radius) {
  PolarPoint polar(static_cast<float>(elev * res + res / 2 - 90), static_cast<float>(azim * res + res / 2 - 180), radius);
  return polar;
}

float angleDifference(float a, float b) {
  float angle = std::fmod(a - b, 360.f);
  return angle >= 0.f ? (angle < 180.f) ? angle : angle - 360.f : (angle >= -180.f) ? angle : angle + 360.f;
}

} // namespace DRONE_NAVIGATION
