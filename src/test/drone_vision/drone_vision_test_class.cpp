#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

#include "camera_wrapper.hpp"
#include "yolo_wrapper.hpp"

#include <iostream>
#include <set>
#include <vector>
#include <string>


namespace DRONE_NAVIGATION {

class YOLOWrapperTestClass : public YOLOWrapper {

public:
  
  YOLOWrapperTestClass() = default;
  ~YOLOWrapperTestClass() = default;

  void runTests() {

    std::string image_path = "src/yolo/dataset/000000017627.jpg";
    cv::Mat image = cv::imread(image_path);

    if (image.empty()) std::cout << "[ERROR] Image could not be loaded!\n";
    else std::cout << "[INFO] Loaded image successfully, image size (" << image.size[0] << " x " << image.size[1] << ")\n";

    cv::imshow("Input preview - press any key to continue", image);
    cv::waitKey(0);

    this->setInput(image);

    if (this->input_.empty()) printf("[ERROR] Could not set input image!\n");
    else std::cout << "[INFO] Successfully cached input image, image size (" << this->input_.size[0] << " x " << this->input_.size[1] << ")\n";

    this->run();

    std::cout << "[INFO] Successfully performed one iference\n";

    std::vector<YOLO_DETECTOR::Detection> results = this->detector_->detect(this->input_);

    std::cout << "[INFO] Numebr of detections (" << results.size() << ")\n";

    std::set<int> found_classes = {};
    for (const auto& det : results) {
      found_classes.insert(det.classId); 
    }
    std::cout << "[INFO] Total distinct classes (" << found_classes.size() << ")\n";

    std::vector<std::string> class_names = YOLO_DETECTOR::getClassNames("src/yolo/labels/coco.names");
    for (const auto& el : found_classes) {
      std::cout << el << " " << class_names[el] <<std::endl;
    }

    cv::Mat output_image;
    image.copyTo(output_image);
    this->detector_->drawBoundingBox(output_image, results);

    cv::imshow("Output preview - press any key to continue", output_image);
    cv::waitKey(0);

    cv::imwrite("src/yolo/dataset/000000017627_out.jpg", output_image);

    Eigen::Vector2i target_loc = this->getOutput();
    std::cout << "[INFO] Target location (" << target_loc.x() << ", " << target_loc.y() << ")\n";
  }

};
    
} // namespace DRONE_NAVIGATION


int main() {

  DRONE_NAVIGATION::YOLOWrapperTestClass yolo_wrapper_test_class = DRONE_NAVIGATION::YOLOWrapperTestClass();
  yolo_wrapper_test_class.runTests();

  return 0;
}