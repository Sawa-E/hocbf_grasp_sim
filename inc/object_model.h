#ifndef HOCBF_OBJECT_MODEL_H
#define HOCBF_OBJECT_MODEL_H

#include <cmath>

// Elasto-plastic contact model (ADR-004)
// K_obj(theta_rel) = K_0                                   if theta_rel < theta_yield
//                  = K_0 * exp(-beta * (theta_rel - theta_yield))  if theta_rel >= theta_yield
// tau_ext = K_obj(theta_rel) * theta_rel

struct object_model
{
  double K_0;            // elastic stiffness [Nm/rad]
  double theta_yield;    // yield point [rad]
  double beta;           // decay rate [1/rad]
  double mass;           // object mass [kg]
  double tau_min;        // minimum grasp torque [Nm]
  double theta_contact;  // contact angle [rad]

  double compute_K_obj(double theta) const
  {
    double theta_rel = theta - theta_contact;
    if (theta_rel <= 0.0) return 0.0;
    if (theta_rel < theta_yield) return K_0;
    return K_0 * std::exp(-beta * (theta_rel - theta_yield));
  }

  double compute_tau_ext(double theta) const
  {
    double theta_rel = theta - theta_contact;
    if (theta_rel <= 0.0) return 0.0;
    double K = compute_K_obj(theta);
    return K * theta_rel;
  }

  double get_theta_rel(double theta) const
  {
    double rel = theta - theta_contact;
    return (rel > 0.0) ? rel : 0.0;
  }
};

#endif // HOCBF_OBJECT_MODEL_H
