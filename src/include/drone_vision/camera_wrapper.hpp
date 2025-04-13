#pragma once

#include "opencv2/opencv.hpp"


namespace DRONE_NAVIGATION {

struct CameraWrapperConfig {};

class CameraWrapper {

public:

  CameraWrapper();
  ~CameraWrapper() = default;

protected:

  CameraWrapperConfig config_ = {};
};

} // namesapce DRONE_NAVIGATION
