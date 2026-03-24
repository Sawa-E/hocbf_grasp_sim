#ifndef MOTIONCONTROL_SYSTEM_CONTROLLER_H
#define MOTIONCONTROL_SYSTEM_CONTROLLER_H
#include <iostream>
#include <unistd.h>
#include <functional>
#include "robot_system.h"
#include "gui.h"
#include "reader.h"
#include "writer.h"
#include "signal_processing.h"
#include "control_timer.h"
#include "environment.h"
#include "controller.h"
#ifdef SIMULATOR
#include "simulator.h"
#include "thread_definition.h"
#endif

class system_controller
{
//using task_method = int (system_controller::*)(long long int);
using ll = long long int;
using task_method = std::function<int(ll)>;
//using task_method = int (*)(long long int);
public:
  const static int FINISH = 1;
  const static int ON = 0;
  std::array<task_method, mc::thread::thread_list_size> tasks_;

  // method
  system_controller()
  :gui_("main window")
  {
    // set default tasks
    for (auto &task : tasks_)
      task = default_task();
    task_registration();
    std::cout << "constructor system controller" << std::endl;
  }

  virtual void calculate_output_command()
  {
    control_.controller[robot_.get_control_mode()](robot_);
    limit_force_output();
    convert_force_to_voltage();
    robot_.increase_control_step();
  }

  virtual void add_joint(const char* filename){ robot_.joints.push_back(filename); }
  virtual void add_end(){ robot_.add_end(); }
  virtual size_t size(){ return robot_.joints.size(); }

  virtual void set_task_takt_time(long long int time, int thread_id)
  {
    robot_.task_takt_times.at(thread_id) = time;
  }

  virtual void add_task_takt_time()
  {
    robot_.task_takt_times.push_back(0);
  }
  virtual void set_timer_sampling_time(long long time, int thread_id)
  {
    robot_.timer_sampling_times.at(thread_id) = time;
  }

  virtual void add_timer_sampling_time()
  {
    robot_.timer_sampling_times.push_back(0);
  }

  virtual void add_reader(reader<double> *reader_ptr)
  {
    std::cout << "added reader" << std::endl;
    readers_ptr_.push_back(reader_ptr);
  }

  virtual void add_writer(writer<double> *writer_ptr)
  {
    std::cout << "added writer" << std::endl;
    writers_ptr_.push_back(writer_ptr);
  }

  virtual robot_system *get_robot()
  {
    return &robot_;
  }

  virtual void close()
  {
    for (size_t i = 0; i < readers_ptr_.size(); ++i)
      readers_ptr_[i]->close();
    for (size_t i = 0; i < writers_ptr_.size(); ++i)
      writers_ptr_[i]->close();
  }
private:
  robot_system  robot_;
  gui           gui_;
  mc::control   control_;
  std::vector<reader<double>*> readers_ptr_;
  std::vector<writer<double>*> writers_ptr_;

#define f_out(n) robot_.joints[(n)].data[mc::output][mc::f]
#define voltage(n) robot_.joints[(n)].data[mc::output][mc::voltage]
#define force_to_voltage(n) robot_.joints[(n)].parameter[mc::force_to_voltage]
#define force_limit(n) robot_.joints[(n)].parameter[mc::force_limit]
#define gear_ratio(n) robot_.joints[(n)].parameter[mc::gear_ratio]
  // method
  void limit_force_output()
  {
    for (size_t i = 0 ; i < size(); ++i)
    {
      if (f_out(i) >= force_limit(i))
        f_out(i) = force_limit(i);
      if (f_out(i) <= -force_limit(i))
        f_out(i) = -force_limit(i);
    }
  }

  void convert_force_to_voltage()
  {
    for (size_t i = 0 ; i < size(); ++i)
    {
      if (robot_.joints[i].parameter[mc::output_inverse] <= 0.5)
      {
        voltage(i) = f_out(i) * force_to_voltage(i) / gear_ratio(i);
      }
      else
      {
        voltage(i) = -1.0 * f_out(i) * force_to_voltage(i) / gear_ratio(i);
      }
    }
  }
#undef f_out
#undef voltage
#undef force_to_voltage
#undef force_limit
#undef gear_ratio
  void task_registration();

  task_method default_task()
  {
    auto ret = [](ll time)
    {
      (void)(time);
      return system_controller::ON;
    };
    return ret;
  }
  void record_line(FILE *fp, robot_system &robot);
  FILE *record_open()
  {
    static std::string dir_name = mc::env::variable["path"];
    FILE *fp;
    long long int current = control_timer::get_micro_time();

    std::string file_name = "";
    if (dir_name != "")
      file_name += dir_name + "/";
    file_name += std::to_string(current);
    file_name += ".csv";
    fp = fopen(file_name.c_str(), "w");
    return fp;
  }

  void update_all_position()
  {
    for(size_t i = 0; i < size(); ++i)
    {
      if (robot_.joints[i].parameter[mc::position_inverse] <= 0.5)
      {
        robot_.joints[i].data[mc::response][mc::x] = robot_.joints[i].data[mc::response][mc::pulse_count];
      }
      else
      {
        robot_.joints[i].data[mc::response][mc::x] = -1.0 * robot_.joints[i].data[mc::response][mc::pulse_count];
      }
    }
  }
};
#endif //MOTIONCONTROL_SYSTEM_CONTROLLER_H
