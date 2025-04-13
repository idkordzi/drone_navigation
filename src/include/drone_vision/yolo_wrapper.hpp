#pragma once

#include <string>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include "yolo_detector.hpp"


namespace DRONE_NAVIGATION {

struct YOLOWrapperConfig {
  unsigned im_width_  = 640;
  unsigned im_height_ = 360;

  unsigned yolo_in_width_  = 0;
  unsigned yolo_in_height_ = 0;

  int   yolo_class_    = 2;
  float yolo_min_conf_ = 0.6f;

  // TODO get path to model/labels from external source (ex. ROS2)
  std::string model_path_  = "install/drone_navigation_ros/share/drone_navigation/yolo/models/yolo11n.onnx"; // "src/yolo/models/yolo11n.onnx"
  std::string labels_path_ = "install/drone_navigation_ros/share/drone_navigation/yolo/labels/coco.names"; // "src/yolo/labels/coco.names"
};

class YOLOWrapper {

public:

  YOLOWrapper();
  ~YOLOWrapper() = default;

  void setInput(const cv::Mat& image);
  Eigen::Vector2i getOutput() const;

  void run();

protected:

  YOLOWrapperConfig config_ = {};

  std::unique_ptr<YOLO_DETECTOR::YOLO12Detector> detector_;

  cv::Mat input_;
  Eigen::Vector2i target_loc_ = {};
};

} // namesapce DRONE_NAVIGATION
