#ifndef HOCBF_QP_H
#define HOCBF_QP_H

#include <algorithm>

// HOCBF-QP safety filter (ADR-005, step1_formulation.md §1-3)
// 1-DOF: QP reduces to clamp(u_nom, u_lb, u_ub)

namespace hocbf {

// h_ub = theta_max - x1 (deformation prevention) → upper bound on u
// u_ub = K_hat*x1 + M*[gamma^2*(theta_max - x1) - 2*gamma*x2 + 2*gamma*theta_max_dot]
inline double compute_u_ub(double x1, double x2, double K_hat, double theta_max,
                           double theta_max_dot, double M, double gamma_pos)
{
  return K_hat * x1
    + M * (gamma_pos * gamma_pos * (theta_max - x1)
           - 2.0 * gamma_pos * x2
           + 2.0 * gamma_pos * theta_max_dot);
}

// h_lb = K_hat*x1 - tau_min (slip prevention) → lower bound on u
// u_lb = K_hat*x1 - M*[2*gamma*x2 + gamma^2*(x1 - tau_min/K_hat)]
inline double compute_u_lb(double x1, double x2, double K_hat, double tau_min,
                           double M, double gamma_force)
{
  if (K_hat <= 1e-6) return 0.0;
  return K_hat * x1
    - M * (2.0 * gamma_force * x2
           + gamma_force * gamma_force * (x1 - tau_min / K_hat));
}

// QP solution: clamp
inline double compute_u_star(double u_nom, double u_lb, double u_ub)
{
  return std::clamp(u_nom, u_lb, u_ub);
}

// Graspability Index
inline double compute_G(double u_ub, double u_lb)
{
  return u_ub - u_lb;
}

// Barrier function values
inline double compute_h_lb(double K_hat, double x1, double tau_min)
{
  return K_hat * x1 - tau_min;
}

inline double compute_h_ub(double theta_max, double x1)
{
  return theta_max - x1;
}

} // namespace hocbf

#endif // HOCBF_QP_H
