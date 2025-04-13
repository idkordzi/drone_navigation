#include "yolo_wrapper.hpp"


namespace DRONE_NAVIGATION {

YOLOWrapper::YOLOWrapper() {

  this->detector_ = std::make_unique<YOLO_DETECTOR::YOLO12Detector>(this->config_.model_path_, this->config_.labels_path_, false);
  this->input_    = cv::Mat(this->config_.yolo_in_height_, this->config_.yolo_in_width_, CV_8UC3, cv::Scalar(0, 0, 0));

  this->target_loc_.fill(-1);
}

void YOLOWrapper::setInput(const cv::Mat& image) {
  image.copyTo(this->input_);
}

Eigen::Vector2i YOLOWrapper::getOutput() const {
  return this->target_loc_;
}

void YOLOWrapper::run() {
  std::vector<YOLO_DETECTOR::Detection> results = this->detector_->detect(this->input_);

  // Get detection with highest confidence 
  unsigned bresult = -1;
  float bconf = 0.0f;
  for (unsigned i = 0; i < results.size(); i++) {
    if (results[i].classId == this->config_.yolo_class_ && results[i].conf > bconf) {
      bconf = results[i].conf;
      bresult = i;
    }
  }
  if (bconf >= this->config_.yolo_min_conf_) {
    this->target_loc_.x() = std::floor(results[bresult].box.x + results[bresult].box.width);
    this->target_loc_.y() = std::floor(results[bresult].box.y + results[bresult].box.height);
  }
  else this->target_loc_.fill(-1);
}

} // namespace DRONE_NAVIGATION
