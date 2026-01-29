cmake_minimum_required(VERSION 3.8)
project(usb_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ==================== 查找依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

# ================= 创建共享库 =================
add_library(camera_capture_node SHARED
  src/camera_capture_node.cpp
)

# ================= 链接ros依赖 =================
ament_target_dependencies(camera_capture_node
  rclcpp
  rclcpp_components
  sensor_msgs
  camera_info_manager
)

# ============== 注册组件节点 =================
rclcpp_components_register_node(camera_capture_node
PLUGIN "usb_camera_driver::CameraCaptureNode"
EXECUTABLE camera_capture_node_exe)


# ================ 安装 =================
# 安装共享库
install(TARGETS camera_capture_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()