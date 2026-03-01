cmake_minimum_required(VERSION 3.8)
project(usb_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# ==================== 查找依赖 ====================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(camera_info_manager REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

# ================= 创建共享库 =================
add_library(usb_camera_node SHARED
  src/usb_camera_node.cpp
)

# ================= 链接头文件 =================
target_include_directories(usb_camera_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)


# ================= 链接ros依赖 =================
ament_target_dependencies(usb_camera_node
  camera_info_manager
  rclcpp
  rclcpp_components

  sensor_msgs
)

# ================= 链接库 =================
target_link_libraries(usb_camera_node
 yaml-cpp
)

# ============== 注册组件节点 =================
rclcpp_components_register_node(usb_camera_node
PLUGIN "usb_camera::USBCameraNode"
EXECUTABLE usb_camera_node_exe)


# ================ 安装 =================
install(TARGETS usb_camera_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()