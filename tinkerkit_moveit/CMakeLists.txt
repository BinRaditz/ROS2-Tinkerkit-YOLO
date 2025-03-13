cmake_minimum_required(VERSION 3.8)
project(tinkerkit_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(tinkerkit_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)



add_executable(moveit_interface src/moveit_interface.cpp)
ament_target_dependencies(moveit_interface rclcpp moveit_ros_planning_interface)
# ament_target_dependencies(moveit_interface1 rclcpp moveit_ros_planning_interface)
# find_package(moveit_visual_tools REQUIRED)

# ament_target_dependencies(
#   moveit_ros_planning_interface
#   moveit_visual_tools
#   rclcpp
# )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  moveit_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
