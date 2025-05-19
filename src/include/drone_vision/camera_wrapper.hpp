#pragma once

#include <cstdlib>
#include <vector>

#include "opencv2/opencv.hpp"

#include "Eigen/Dense"


namespace DRONE_NAVIGATION {

struct CameraWrapperConfig {
  
  // camera params
  unsigned im_width = 640;
  unsigned im_height = 360;
};

struct PointXYZD {
  PointXYZD() : x(0.0f), y(0.0f), z(0.0f), distance(0.0f) {}
  PointXYZD(float _x, float _y, float _z, float _d) : x(_x), y(_y), z(_z), distance(_d) {}
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float distance = 0.0f;
};

struct PointCloudFlatten {
  PointCloudFlatten() {};
  PointCloudFlatten(int _w, int _h) : width(_w), height(_h) {cloud.reserve(width*height);}

  unsigned width = 0;
  unsigned height = 0;
  std::vector<PointXYZD> cloud = {};
};

class CameraWrapper {

public:

  CameraWrapper();
  CameraWrapper(CameraWrapperConfig config);
  ~CameraWrapper() = default;

  cv::Mat getColorImage();
  cv::Mat getDepthImage();
  PointCloudFlatten getPointCloud();

protected:

  void initialize();

  CameraWrapperConfig config_ = {};
};

} // namesapce DRONE_NAVIGATION
