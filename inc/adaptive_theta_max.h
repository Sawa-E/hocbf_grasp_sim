#ifndef HOCBF_ADAPTIVE_THETA_MAX_H
#define HOCBF_ADAPTIVE_THETA_MAX_H

#include <cmath>
#include <algorithm>

// Adaptive theta_max (ADR-005, step1_formulation.md §1-4)
// RFOB-triggered adaptation based on dK_obj_hat/dt

struct adaptive_theta_max
{
  double theta_max;
  double theta_max_dot;

  adaptive_theta_max(double theta_max_0, double theta_max_min, double theta_max_ceiling,
                     double alpha_relax, double alpha_shrink_nominal,
                     double epsilon, double epsilon_danger)
    : theta_max(theta_max_0), theta_max_dot(0.0),
      theta_max_min_(theta_max_min), theta_max_ceiling_(theta_max_ceiling),
      alpha_relax_(alpha_relax), alpha_shrink_nominal_(alpha_shrink_nominal),
      epsilon_(epsilon), epsilon_danger_(epsilon_danger)
  {}

  // u_ub_static: u_ub with theta_max_dot=0, u_lb: current lower bound
  void update(double dK_dt, double u_ub_static, double u_lb, double M, double gamma_pos, double dt)
  {
    double abs_dK = std::fabs(dK_dt);

    // dynamic shrink rate upper bound (safety condition (i))
    double feasibility_margin = u_ub_static - u_lb;
    double alpha_shrink_max = (feasibility_margin > 0.0 && M * 2.0 * gamma_pos > 0.0)
      ? feasibility_margin / (M * 2.0 * gamma_pos)
      : 0.0;
    double alpha_shrink = std::min(alpha_shrink_nominal_, alpha_shrink_max);

    if (abs_dK < epsilon_)
    {
      // stable: relax
      theta_max_dot = alpha_relax_;
    }
    else if (dK_dt < -epsilon_danger_)
    {
      // rapid decrease: shrink
      theta_max_dot = -alpha_shrink;
    }
    else
    {
      // transition zone: hold
      theta_max_dot = 0.0;
    }

    theta_max += theta_max_dot * dt;
    theta_max = std::clamp(theta_max, theta_max_min_, theta_max_ceiling_);
  }

  void reset(double theta_max_0)
  {
    theta_max = theta_max_0;
    theta_max_dot = 0.0;
  }

private:
  double theta_max_min_;
  double theta_max_ceiling_;
  double alpha_relax_;
  double alpha_shrink_nominal_;
  double epsilon_;
  double epsilon_danger_;
};

#endif // HOCBF_ADAPTIVE_THETA_MAX_H
