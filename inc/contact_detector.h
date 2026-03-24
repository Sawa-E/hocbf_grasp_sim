#ifndef HOCBF_CONTACT_DETECTOR_H
#define HOCBF_CONTACT_DETECTOR_H

#include <cmath>
#include <vector>

// Contact detection: dual condition AND (ADR-008)
// Condition A: f_dis_hat > mu + k_sigma * sigma (adaptive threshold)
// Condition B: K_obj_hat > K_contact_threshold

struct contact_detector
{
  bool detected;
  double theta_hold;

  contact_detector(int N, double k_sigma, double K_contact_threshold)
    : detected(false), theta_hold(0.0),
      N_(N), k_sigma_(k_sigma), K_th_(K_contact_threshold),
      buf_(N, 0.0), idx_(0), count_(0), sum_(0.0), sum_sq_(0.0)
  {}

  void update(double f_dis_hat, double K_obj_hat, double theta_current)
  {
    if (detected) return; // irreversible

    // update ring buffer for moving average/std
    double old_val = buf_[idx_];
    buf_[idx_] = f_dis_hat;
    idx_ = (idx_ + 1) % N_;

    sum_ += f_dis_hat - old_val;
    sum_sq_ += f_dis_hat * f_dis_hat - old_val * old_val;
    if (count_ < N_) count_++;

    if (count_ < N_) return; // not enough samples yet

    double mu = sum_ / N_;
    double var = sum_sq_ / N_ - mu * mu;
    double sigma = (var > 0.0) ? std::sqrt(var) : 0.0;

    bool cond_a = (f_dis_hat > mu + k_sigma_ * sigma);
    bool cond_b = (K_obj_hat > K_th_);

    if (cond_a && cond_b)
    {
      detected = true;
      theta_hold = theta_current;
    }
  }

  void reset()
  {
    detected = false;
    theta_hold = 0.0;
    idx_ = 0;
    count_ = 0;
    sum_ = 0.0;
    sum_sq_ = 0.0;
    std::fill(buf_.begin(), buf_.end(), 0.0);
  }

private:
  int N_;
  double k_sigma_;
  double K_th_;
  std::vector<double> buf_;
  int idx_;
  int count_;
  double sum_;
  double sum_sq_;
};

#endif // HOCBF_CONTACT_DETECTOR_H
