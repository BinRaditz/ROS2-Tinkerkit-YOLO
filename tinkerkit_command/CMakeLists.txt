cmake_minimum_required(VERSION 3.8)
project(tinkerkit_command)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinkerkit_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tinkerkit_msgs REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Executable sources and dependencies
set(EXECUTABLES
  hello_moveit
  approach_retreat
  cartesian_path
  angle_conversion
  move_group_interface_tutorial
  tcp_broadcast
  robotarm_controller
  pick_and_place
)

set(SOURCES
  src/hello_moveit.cpp
  src/approach_retreat.cpp
  src/cartesian_path.cpp
  src/angle_conversion.cpp
  src/move_group_interface_tutorial.cpp
  src/tcp_broadcast.cpp
  src/robotarm_controller.cpp
  src/pick_and_place.cpp
)

set(DEPENDENCIES
  rclcpp
  moveit_ros_planning_interface
  tinkerkit_msgs
  tf2
  tf2_ros
  geometry_msgs
)

# Create executables and link dependencies
list(LENGTH EXECUTABLES EXEC_COUNT)
math(EXPR MAX_INDEX "${EXEC_COUNT} - 1")

foreach(i RANGE ${MAX_INDEX})
  list(GET EXECUTABLES ${i} EXEC_NAME)
  list(GET SOURCES ${i} SRC_FILE)
  
  add_executable(${EXEC_NAME} ${SRC_FILE})
  ament_target_dependencies(${EXEC_NAME} ${DEPENDENCIES})
endforeach()

# Installation
install(
  DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
