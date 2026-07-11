cmake_minimum_required(VERSION 3.8)
project(lite6_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成 Action 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/IndustrialMotion.action"
  DEPENDENCIES action_msgs geometry_msgs
)

ament_package()