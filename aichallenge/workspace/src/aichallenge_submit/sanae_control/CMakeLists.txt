cmake_minimum_required(VERSION 3.8)
project(sanae_control)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_pd_controller
  src/simple_pd_controller.cpp
)
ament_auto_add_executable(sliding_mode_controller
  src/sliding_mode_controller.cpp
)
ament_auto_add_executable(pid_controller
  src/pid_controller.cpp
)

ament_auto_package()