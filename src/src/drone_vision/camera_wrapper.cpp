#include "camera_wrapper.hpp"


namespace DRONE_NAVIGATION {

CameraWrapper::CameraWrapper() {
  this->initialize();
}

CameraWrapper::CameraWrapper(CameraWrapperConfig config) : config_(config) {
  this->initialize();
}

void CameraWrapper::initialize() {
  // @TODO
}

cv::Mat CameraWrapper::getColorImage() {
  return cv::Mat(this->config_.im_height, this->config_.im_width, CV_8UC3);
}

cv::Mat CameraWrapper::getDepthImage() {
  return cv::Mat(this->config_.im_height, this->config_.im_width, CV_32FC1);
}

PointCloudFlatten CameraWrapper::getPointCloud() {
  return PointCloudFlatten(this->config_.im_height, this->config_.im_width);
}

} // namespace DRONE_NAVIGATION
