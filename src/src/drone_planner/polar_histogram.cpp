#include "polar_histogram.hpp"


namespace DRONE_NAVIGATION {

PolarHistogram::PolarHistogram(int res)
: resolution_{res}, azim_dim_{360 / resolution_}, elev_dim_{180 / resolution_}, distance_(elev_dim_, azim_dim_)
{
  this->clear();
}

void PolarHistogram::upsample() {
  if (this->resolution_ != CELL_SIZE * 2) {
    throw std::logic_error("Invalid use of function upsample(). This function can only be used on a half resolution histogram.");
  }
  this->resolution_ = this->resolution_ / 2;
  this->azim_dim_ = 2 * this->azim_dim_;
  this->elev_dim_ = 2 * this->elev_dim_;
  Eigen::MatrixXf new_histogram(this->elev_dim_, this->azim_dim_);

  for (int i = 0; i < this->elev_dim_; ++i) {
    for (int j = 0; j < this->azim_dim_; ++j) {
      int i_lowres = floor(i / 2);
      int j_lowres = floor(j / 2);
      new_histogram(i, j) = this->distance_(i_lowres, j_lowres);
    }
  }
  this->distance_ = new_histogram;
}

void PolarHistogram::downsample() {
  if (this->resolution_ != CELL_SIZE) {
    throw std::logic_error("Invalid use of function downsample(). This function can only be used on a full resolution histogram.");
  }
  this->resolution_ = 2 * this->resolution_;
  this->azim_dim_ = this->azim_dim_ / 2;
  this->elev_dim_ = this->elev_dim_ / 2;
  Eigen::MatrixXf new_histogram(elev_dim_, azim_dim_);

  for (int i = 0; i < this->elev_dim_; ++i) {
    for (int j = 0; j < this->azim_dim_; ++j) {
      int i_high_res = 2 * i;
      int j_high_res = 2 * j;
      new_histogram(i, j) = this->distance_.block(i_high_res, j_high_res, 2, 2).mean();
    }
  }
  this->distance_ = new_histogram;
}

void PolarHistogram::clear() { this->distance_.fill(0.0f); }

bool PolarHistogram::isEmpty() const {
  bool is_empty = true;
  for (int e = 0; (e < elev_dim_) && (is_empty); e++) {
    for (int z = 0; (z < azim_dim_) && (is_empty); z++) {
      if (this->distance_(e, z) > FLT_MIN) is_empty = false;
    }
  }
  return is_empty;
}

} // namespace DRONE_NAVIGATION
