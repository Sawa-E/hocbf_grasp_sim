#include <iostream>
#include "control_timer.h"
#include "thread_controller.h"
#ifdef PCI_MODE
# include "contec_counter.h"
# include "contec_da.h"
#elif defined(SIMULATOR)
# include "simulator.h"
#endif

using ll=long long int;

int main()
{
  ll sample_frequency = static_cast<ll>(atoi(mc::env::variable["sample_frequency"].c_str()));
  system_controller motion_control_system;

  // Grasp hand: 2 fingers (joint 0 = finger motor, joint 1 = finger motor)
  motion_control_system.add_joint("../config/blue_scara/4018_finger.json");
  motion_control_system.add_joint("../config/blue_scara/4018_finger.json");
  motion_control_system.add_end();

#ifdef PCI_MODE
  contec_counter<double> counter1(0, 2, motion_control_system.get_robot(), "../config/contec_counter1.json");
  contec_da<double> da(0, 2, motion_control_system.get_robot(), "../config/contec_da1.json");
  motion_control_system.add_reader(&counter1);
  motion_control_system.add_writer(&da);
#elif defined(SIMULATOR)
  robot_system virtual_robot;
  virtual_robot.add_joint("../config/blue_scara/4018_finger.json");
  virtual_robot.add_joint("../config/blue_scara/4018_finger.json");
  virtual_robot.add_end();

  simulator<double> sim(0, virtual_robot.joints.size(), motion_control_system.get_robot(), &virtual_robot);

  // Object model for grasp simulation (PET bottle full, ADR-004)
  object_model obj;
  obj.K_0 = 3.0;
  obj.theta_yield = 0.05;
  obj.beta = 30.0;
  obj.mass = 0.500;
  obj.tau_min = 0.025;
  obj.theta_contact = 0.35;
  sim.set_object_model(obj, 0);  // joint 0 = active finger

  motion_control_system.add_reader(&sim);
  motion_control_system.add_writer(&sim);
#endif

  thread_controller motion_control_threads(sample_frequency, &motion_control_system);
  motion_control_threads.thread_registeration();
  motion_control_threads.start_timer();
  return 0;
}
