cmake_minimum_required(VERSION 3.10)
project(atri_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== 查找依赖 ====================
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)

# ================= 生成消息接口 =================
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ColorBlock.msg"
  "msg/ColorBlockArray.msg"
  "msg/Rune.msg"
  "msg/TimeInfo.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs)

ament_package()
