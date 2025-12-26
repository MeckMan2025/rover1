cmake_minimum_required(VERSION 3.8)
project(gnss_health_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssHealth.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python files directly
install(DIRECTORY gnss_health_monitor/
  DESTINATION lib/python3/dist-packages/gnss_health_monitor
  USE_SOURCE_PERMISSIONS
)

# Create executable script
install(PROGRAMS
  gnss_health_monitor/gnss_health_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gnss_health_monitor_node
)

# Install resources for package discovery
install(FILES resource/gnss_health_monitor
  DESTINATION share/ament_index/resource_index/packages
)

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()