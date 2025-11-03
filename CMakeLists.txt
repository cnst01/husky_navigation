cmake_minimum_required(VERSION 3.8)
project(husky_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY 
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_goal.py
  scripts/send_waypoints.py
  scripts/waypoints_demo.py
  scripts/waypoint_state_machine.py
  scripts/get_waypoints_state.py
  scripts/navigate_waypoints_state.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()