cmake_minimum_required(VERSION 3.8)
project(rover1_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PatrolStatus.msg"
  "srv/SavePath.srv"
  "srv/ListPaths.srv"
  "srv/StartPatrol.srv"
  "srv/SaveMapSnapshot.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Install Python executables
install(PROGRAMS
  rover1_patrol/waypoint_recorder.py
  rover1_patrol/patrol_manager.py
  rover1_patrol/map_snapshot.py
  rover1_patrol/velocity_scaler.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
