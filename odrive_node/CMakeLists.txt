cmake_minimum_required(VERSION 3.8)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlMessage.msg"
  "msg/ControllerStatus.msg"
  "msg/ODriveStatus.msg"
  "srv/AxisState.srv"
  "srv/SetConfigs.srv"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)
include_directories(../odrive_base/include)

add_executable(odrive_can_node 
  ../odrive_base/src/epoll_event_loop.cpp
  ../odrive_base/src/socket_can.cpp
  src/odrive_can_node.cpp
  src/main.cpp
  include/odrive_can_node.hpp)

ament_target_dependencies(odrive_can_node
  rclcpp
  std_msgs
  std_srvs
)

target_compile_features(odrive_can_node PRIVATE cxx_std_20)

install(
  TARGETS odrive_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odrive_can_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_odrive_protocol_policy
    test/test_odrive_protocol_policy.cpp
  )
  target_include_directories(test_odrive_protocol_policy PRIVATE include)

  ament_add_gtest(
    test_epoll_event_loop
    test/test_epoll_event_loop.cpp
    ../odrive_base/src/epoll_event_loop.cpp
  )
  target_include_directories(test_epoll_event_loop PRIVATE ../odrive_base/include)
endif()

ament_package()
