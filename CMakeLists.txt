cmake_minimum_required(VERSION 3.5)
project("HOCBF_GraspSim")
set(TARGET_NAME "hocbf_sim")

if (APPLE)
  find_package(OpenGL REQUIRED)
  find_library(COCOA_LIBRARY Cocoa REQUIRED)
  find_library(IOKIT_LIBRARY IOKit REQUIRED)
  find_library(COREVID_LIBRARY CoreVideo REQUIRED)
  include_directories(/usr/local/include)
endif(APPLE)

include_directories(./inc/)
include_directories(./lib/)
include_directories(./lib/imgui)
add_subdirectory(lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wformat -Wextra -O3")

# Switch between simulation and real hardware:
#add_definitions(-DPCI_MODE)
add_definitions(-DSIMULATOR)

# if not defined then timer will not wait.
add_definitions(-DREALTIME)

add_executable(${TARGET_NAME}
  ./src/main.cc
  ./src/control_timer.cc
  ./src/thread_controller.cc
  ./src/system_controller.cc
  ./src/gui.cc
  ./src/load_shader.cc
  ./src/controller.cc
  inc/thread_definition.h
)

if (APPLE)
  target_link_libraries(${TARGET_NAME} -lpthread ${OPENGL_GL_LIBRARY} ${COCOA_LIBRARY} ${COREVID_LIBRARY} ${IOKIT_LIBRARY} -L/usr/local/lib/ -lglfw -lm -ldl GUI)
elseif (UNIX)
  target_link_libraries(${TARGET_NAME} GUI -lpthread -lGL -lrt -lglfw -lm -ldl stdc++fs -lpci)
endif()
