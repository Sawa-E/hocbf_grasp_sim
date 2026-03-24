#ifndef HOCBF_RFOB_H
#define HOCBF_RFOB_H

// RFOB: Reaction Force Observer (ADR-007)
// K_hat = f_dis_hat / theta_rel, smoothed by LPF

struct rfob
{
  double K_obj_hat;
  double dK_obj_hat_dt;

  rfob() : K_obj_hat(0.0), dK_obj_hat_dt(0.0), buf_(0.0), K_prev_(0.0), dk_buf_(0.0) {}

  void update(double f_dis_hat, double theta_rel, double dt, double g_rfob, double g_dk)
  {
    double K_prev = K_obj_hat;

    if (theta_rel > theta_rel_min_)
    {
      double K_raw = f_dis_hat / theta_rel;
      // LPF: buf += dt * (K_raw - K_hat); K_hat = g * buf
      buf_ += dt * (K_raw - K_obj_hat);
      K_obj_hat = g_rfob * buf_;
    }

    // dK/dt via pseudo-differential
    dk_buf_ += dt * dK_obj_hat_dt;
    dK_obj_hat_dt = g_dk * (K_obj_hat - dk_buf_);

    K_prev_ = K_prev;
  }

  void reset()
  {
    K_obj_hat = 0.0;
    dK_obj_hat_dt = 0.0;
    buf_ = 0.0;
    K_prev_ = 0.0;
    dk_buf_ = 0.0;
  }

private:
  double buf_;
  double K_prev_;
  double dk_buf_;
  static constexpr double theta_rel_min_ = 0.001; // zero-division guard
};

#endif // HOCBF_RFOB_H
