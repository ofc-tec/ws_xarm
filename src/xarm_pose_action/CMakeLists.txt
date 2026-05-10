cmake_minimum_required(VERSION 3.8)
project(xarm_pose_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SetJoints.action"
  "action/SetPose.action"
  DEPENDENCIES geometry_msgs
)

add_executable(set_pose_action_server src/set_pose_action_server.cpp)
target_compile_features(set_pose_action_server PUBLIC cxx_std_17)
ament_target_dependencies(set_pose_action_server
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(set_pose_action_server "${cpp_typesupport_target}")

add_executable(set_joints_action_server src/set_joints_action_server.cpp)
target_compile_features(set_joints_action_server PUBLIC cxx_std_17)
ament_target_dependencies(set_joints_action_server
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
)
target_link_libraries(set_joints_action_server "${cpp_typesupport_target}")

install(TARGETS
  set_joints_action_server
  set_pose_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
