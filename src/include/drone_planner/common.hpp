#pragma once

#include <cmath>
#include "Eigen/Dense"


namespace DRONE_NAVIGATION {

// constants

#define PI_F 3.14159265358979323846f

constexpr float DEG_TO_RAD = PI_F / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI_F;


// structures

struct PolarPoint {
  PolarPoint(float e_, float z_, float r_) : elev(e_), azim(z_), radi(r_) {};
  PolarPoint() : elev(0.0f), azim(0.0f), radi(0.0f) {};
  float elev = 0.0f;
  float azim = 0.0f;
  float radi = 0.0f;
};

struct FOV {
  FOV() : yaw_deg(0.0f), pitch_deg(0.0f), h_fov_deg(0.0f), v_fov_deg(0.0f) {};
  FOV(float y_, float p_, float h_, float v_) : yaw_deg(y_), pitch_deg(p_), h_fov_deg(h_), v_fov_deg(v_) {};
  float yaw_deg   = 0.0f;
  float pitch_deg = 0.0f;
  float h_fov_deg = 0.0f;
  float v_fov_deg = 0.0f;
};


// functions

inline float clip(float val, float lo_lim = -INFINITY, float hi_lim = INFINITY) {
  return std::min(std::max(val, lo_lim), hi_lim);
}

inline float sqr(float x) {return x*x;}

inline float wrapRangeRad(float angle) {return (angle - 2.0f * PI_F * std::floor(angle / (2.0f * PI_F) + 0.5f)); }
inline float wrapRangeDeg(float angle) {return (angle - 360.0f * std::floor(angle / 360.0f + 0.5f)); }

bool pointInsideFOV(const FOV& fov, const PolarPoint& polar);

PolarPoint convertCartesianToPolar(const Eigen::Vector3f& point, const Eigen::Vector3f& origin);
Eigen::Vector3f convertPolarToCartesian(const PolarPoint& polar, const Eigen::Vector3f& origin);

void wrapPolar(PolarPoint& polar);

Eigen::Vector2i convertPolarToHistogramIndex(const PolarPoint& polar, int res);
PolarPoint convertHistogramIndexToPolar(int elev, int azim, int res, float radius);

float angleDifference(float a, float b);

} // namespace DRONE_NAVIGATION
