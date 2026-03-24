#ifndef MOTIONCONTROL_SIMULATOR_H
#define MOTIONCONTROL_SIMULATOR_H
#include <iostream>
#include "reader.h"
#include "writer.h"
#include "robot_system.h"
#include "object_model.h"

//class simulator :public reader
template<typename T>
class simulator : public writer<T>, public reader<T>
{
public:
  // data
  const static int SUCCESS=0;
  const static int FAIL=1;

  // method
  simulator(size_t start_channel, size_t number_of_channel, robot_system *robot, robot_system *simulated_robot, double dt = 0.0001)
  :writer<T>(start_channel, number_of_channel, robot, mc::f),
  reader<T>(start_channel, number_of_channel, robot),
  simulated_robot_ptr_(simulated_robot),
  dt_(dt)
  {
    std::cout << "simulator constructor" << std::endl;
  }

  int open()
  {
    std::cout << "simulator open" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
    return SUCCESS;
  }

  int read()
  {
    for (size_t i = 0; i < reader<T>::number_of_channel_; ++i)
    {
      *(reader<T>::channel_ptr(i)) = simulated_robot_ptr_->joints.at(reader<T>::start_channel_ + i).data[mc::response][reader<T>::read_state_];
    }
    return SUCCESS;
  }

  int read(size_t idx)
  {
    std::cout << "simulator read one" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
    (void)(idx);
    //return reader::len();
    return reader<T>::number_of_channel_;
  }

  void reset()
  {
    std::cout << "simulator reset" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
  }

  void initialize()
  {
    std::cout << "simulator initialize" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
  }

  int write()
  {
    writer<T>::write();
    return SUCCESS;
  }

  int write(size_t idx)
  {
    std::cout << "simulator write one" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
    (void)(idx);
    return SUCCESS;
  }

  int write_buf()
  {
    for (size_t i = 0; i < writer<T>::number_of_channel_; ++i)
    {
       simulated_robot_ptr_->joints.at(writer<T>::start_channel_ + i).data[mc::response][writer<T>::write_state_] = *(writer<T>::channel_ptr(i));
    }
    return SUCCESS;
  }

  #define   M(n) simulated_robot_ptr_->joints.at(n).parameter[mc::mass]
  #define   f_sim(n) simulated_robot_ptr_->joints.at(n).data[mc::response][mc::f]
  #define ddx_sim(n) simulated_robot_ptr_->joints.at(n).data[mc::response][mc::ddx]
  #define  dx_sim(n) simulated_robot_ptr_->joints.at(n).data[mc::response][mc::dx]
  #define   x_sim(n) simulated_robot_ptr_->joints.at(n).data[mc::response][mc::x]
  int flush()
  {
    // ddx = (u - tau_ext) / M,  dx += ddx * dt,  x += dx * dt
    for (size_t i = 0; i < writer<double>::number_of_channel_; ++i)
    {
      int idx = i + writer<double>::start_channel_;
      double u = f_sim(idx);
      double tau_ext = 0.0;
      if (has_object_ && idx == obj_joint_idx_)
        tau_ext = obj_.compute_tau_ext(x_sim(idx));
      ddx_sim(idx)   = (u - tau_ext) / M(idx);
       dx_sim(idx)  += ddx_sim(idx) * dt_;
        x_sim(idx)  += dx_sim(idx) * dt_;
      // pulse_count にコピー（read() が pulse_count を読むため）
      // position_inverse=1.0 の場合 update_all_position() が x = -pulse_count とするため
      // -x_sim を格納して x = -(-x_sim) = x_sim となるようにする
      double sign = (simulated_robot_ptr_->joints.at(idx).parameter[mc::position_inverse] > 0.5) ? -1.0 : 1.0;
      simulated_robot_ptr_->joints.at(idx).data[mc::response][mc::pulse_count] = sign * x_sim(idx);
    }
    return SUCCESS;
  }
  #undef   M
  #undef   f_sim
  #undef ddx_sim
  #undef  dx_sim
  #undef   x_sim

  void zero()
  {
    std::cout << "simulator zero" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
  }

  void close()
  {
    std::cout << "simulator close" << std::endl;
    std::cout << "\tThis is not implemeted, so this does nothing." << std::endl;
  }

  void set_object_model(object_model obj, int joint_idx)
  {
    obj_ = obj;
    obj_joint_idx_ = joint_idx;
    has_object_ = true;
  }

private:
  robot_system *simulated_robot_ptr_;
  double dt_;
  object_model obj_;
  int obj_joint_idx_ = 0;
  bool has_object_ = false;
};
#endif //MOTIONCONTROL_SIMULATOR_H
