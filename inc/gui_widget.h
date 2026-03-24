#ifndef MOTIONCONTROL_GUI_WIDGET_H
#define MOTIONCONTROL_GUI_WIDGET_H
#include <iostream>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "robot_system.h"
#include "implot.h"
#include "enum_helper.h"
#include "thread_definition.h"

namespace mc {
  struct scrolling_buffer
  {
    int max_size;
    int offset;
    ImVector<ImVec2> data;
    scrolling_buffer(int max_size = 2000) {
      this->max_size = max_size;
      offset  = 0;
      data.reserve(max_size);
    }
    void add_point(float x, float y) {
      if (data.size() < max_size)
        data.push_back(ImVec2(x,y));
      else {
        data[offset] = ImVec2(x,y);
        offset =  (offset + 1) % max_size;
      }
    }
    void erase() {
      if (data.size() > 0) {
        data.shrink(0);
        offset  = 0;
      }
    }
  };

  class widget
  {
  public:
    static void buttons(robot_system *robot_system_ptr)
    {
      (void)(robot_system_ptr);
      ImGui::Columns(3, "button");  // 3-ways, no border
      if(ImGui::Button("start", ImVec2(-1.0f, 0.0f))){
        std::cout << "start clicked" << std::endl ;
      }
      ImGui::NextColumn();
      if(ImGui::Button("stop", ImVec2(-1.0f, 0.0f))){
        std::cout << "stop clicked" << std::endl ;
      }
      ImGui::NextColumn();
      if(ImGui::Button("close", ImVec2(-1.0f, 0.0f))){
        std::cout << "close clicked" << std::endl ;
      }
    }

    static void callback(ImGuiInputTextCallbackData* data)
    {
      std::cout << "callbacked is called" << std::endl;
    }

    static void form(robot_system *robot_system_ptr)
    {
      (void)(robot_system_ptr);
      //static std::unordered_map<std::string, float> dict;
      //ImGui::Columns(3, "form");  // 3-ways, no border
      static char key_buf[32] = ""; ImGui::InputText("key", key_buf, sizeof(key_buf));
      static char val_buf[32] = ""; ImGui::InputText("value", val_buf, sizeof(val_buf));
      if(ImGui::Button("set", ImVec2(-1.0f, 0.0f))){
        robot_system_ptr->set_to_dict(key_buf, std::atof(val_buf));
        std::cout << "key: " << key_buf << ", value: " << robot_system_ptr->get_from_dict(key_buf) << std::endl;
      }
    }

    static void display_robot(robot_system *robot_system_ptr)
    {
      ImGui::Columns(7, "takt time");  // 3-ways, no border
      ImGui::Text("Mode: %d ", robot_system_ptr->get_control_mode());
      ImGui::NextColumn();
      ImGui::Text("Time: %lld [us]",robot_system_ptr->task_takt_times[mc::thread::compute_engine]);
      ImGui::NextColumn();
      ImGui::Text("Time: %lld [us]",robot_system_ptr->timer_sampling_times[mc::thread::compute_engine]);
      ImGui::NextColumn();
      ImGui::Text("Time: %lld [us]",robot_system_ptr->task_takt_times[mc::thread::read_sensor]);
      ImGui::NextColumn();
      ImGui::Text("Time: %lld [us]",robot_system_ptr->timer_sampling_times[mc::thread::read_sensor]);
      ImGui::NextColumn();
      ImGui::Text("Time: %lld [us]",robot_system_ptr->task_takt_times[mc::thread::write_output]);
      ImGui::NextColumn();
      ImGui::Text("Time: %lld [us]",robot_system_ptr->timer_sampling_times[mc::thread::write_output]);
      ImGui::Separator();

      ImGui::Columns(4+1, "motion_header_column");  // 3-ways, no border
      ImGui::Text(" ");
      ImGui::NextColumn();
      ImGui::Text("position");
      ImGui::NextColumn();
      ImGui::Text("velocity");
      ImGui::NextColumn();
      ImGui::Text("force");
      ImGui::NextColumn();
      ImGui::Text("damper");
      ImGui::NextColumn();
      ImGui::Columns(1);
      ImGui::Separator();

      for (size_t i=0 ;i<robot_system_ptr->joints.size(); ++i)
      {
        ImGui::Columns(5, "motion_display_column");
        ImGui::Text("joint %d", i);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).data[mc::response][mc::x]);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).data[mc::response][mc::dx]);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).data[mc::response][mc::f_dis]);
        ImGui::NextColumn();
        ImGui::Text("%+05.5lf", robot_system_ptr->joints.at(i).parameter[mc::damper]);
        ImGui::NextColumn();
        ImGui::Columns(1);
      }
    }

    static void control_mode_selection(robot_system *robot_system_ptr)
    {
      ImGui::Columns(mc::control_mode_size, "control mode");  // 3-ways, no border

      for (int i = 0; i < mc::control_mode_size; ++i)
      {
        const auto mode = static_cast<mc::control_mode>(i);
        std::string mode_name = mc::enum_helper::name(mode);
        if(ImGui::Button(mode_name.c_str(), ImVec2(-1.0f, 0.0f)))
        {
          robot_system_ptr->control_mode_request = static_cast<mc::control_mode>(i);
          std::cout << "mode [ " << mode_name << "] is seleted" << std::endl ;
        }
        ImGui::NextColumn();
      }
    }

    static void record_mode_selection(robot_system *robot_system_ptr)
    {
      bool is_recording = robot_system_ptr->record_mode_state == mc::recording;

      if (!is_recording)
      {
        ImGui::Columns(1, "record mode");  // 3-ways, no border
        if(ImGui::Button("record", ImVec2(-1.0f, 0.0f))){
          robot_system_ptr->record_mode_state = mc::record_requesting;
          std::cout << "request record start" << std::endl ;
        }
      }
      else if (is_recording)
      {
        ImGui::Columns(2, "record mode");  // 3-ways, no border
        if(ImGui::Button("stop", ImVec2(-1.0f, 0.0f))){
          robot_system_ptr->record_mode_state = mc::record_stop_requesting;
          std::cout << "request record stop" << std::endl ;
        }
        if(ImGui::Button("pose", ImVec2(-1.0f, 0.0f))){
          robot_system_ptr->record_mode_state = mc::record_pose_requesting;
          std::cout << "request record pose" << std::endl ;
        }
      }
    }

#define   x_res(n) robot_system_ptr->joints[(n)].data[mc::response][mc::x]
#define  dx_res(n) robot_system_ptr->joints[(n)].data[mc::response][mc::dx]
#define ddx_res(n) robot_system_ptr->joints[(n)].data[mc::response][mc::ddx]
#define   f_dis(n) robot_system_ptr->joints[(n)].data[mc::response][mc::f_dis]
#define   f_ref(n) robot_system_ptr->joints[(n)].data[mc::reference][mc::f]
#define   f_out(n) robot_system_ptr->joints[(n)].data[mc::output][mc::f]
#define   k_p(n) robot_system_ptr->joints[(n)].parameter[mc::k_p]
#define   k_v(n) robot_system_ptr->joints[(n)].parameter[mc::k_v]
#define   k_f(n) robot_system_ptr->joints[(n)].parameter[mc::k_f]
#define   M(n) robot_system_ptr->joints[(n)].parameter[mc::mass]
#define   D(n) robot_system_ptr->joints[(n)].parameter[mc::damper]
#define   K(n) robot_system_ptr->joints[(n)].parameter[mc::spring]
#define   param(n) robot_system_ptr->joints[(n)].parameter
    static void plot_state(robot_system *robot_system_ptr)
    {
      static std::vector<scrolling_buffer> sdata_vec(4);
      static float t = 0;
      static float history = 2.0f;

      t += ImGui::GetIO().DeltaTime;
      sdata_vec[0].add_point(t, x_res(11));
      sdata_vec[1].add_point(t, f_dis(11));
      //// Plot motion data
      ImPlot::SetNextPlotLimitsX(t - history, t + history, ImGuiCond_Always);
      ImPlot::SetNextPlotLimitsY(0,1);
      if (ImPlot::BeginPlot("position", "time[sec]", "x[m]", ImVec2(-1,500)))
      {
        ImPlot::PlotLine("x_11", &sdata_vec[0].data[0].x, &sdata_vec[0].data[0].y, sdata_vec[0].data.size(), sdata_vec[0].offset, 2*sizeof(float));
        ImPlot::PlotLine("f_11", &sdata_vec[1].data[0].x, &sdata_vec[1].data[0].y, sdata_vec[1].data.size(), sdata_vec[1].offset, 2*sizeof(float));
        ImPlot::EndPlot();
      }
    }
#undef   x_res
#undef  dx_res
#undef ddx_res
#undef   f_dis
#undef   f_ref
#undef   f_out
#undef   k_p
#undef   k_v
#undef   k_f
#undef   M
#undef   D
#undef   K
#undef   param
  };
} // namespace mc
#endif //MOTIONCONTROL_GUI_WIDGET_H
