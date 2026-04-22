cmake_minimum_required(VERSION 3.10)
project(atri_serial_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== 查找依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(atri_interfaces REQUIRED)

find_package(serial_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)

# ================= 创建共享库 =================
add_library(atri_serial_driver SHARED src/atri_serial_driver.cpp)

add_library(crc SHARED src/crc.cpp)

# ================= 设置头文件目录 =================
target_include_directories(
  atri_serial_driver
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(
  crc PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
             $<INSTALL_INTERFACE::${CMAKE_CURRENT_SOURCE_DIR}/include>)

# ================= 链接ros依赖 =================
ament_target_dependencies(
  atri_serial_driver
  rclcpp
  rclcpp_components
  tf2_geometry_msgs
  atri_interfaces
  serial_driver
  tf2)

# =============== 链接库 ================
target_link_libraries(atri_serial_driver crc Eigen3::Eigen)

# ============== 注册组件节点 =================
rclcpp_components_register_node(
  atri_serial_driver PLUGIN "atri_serial_driver::ATRISerialDriver" EXECUTABLE
  atri_serial_driver_exe)

# ================ 安装 =================
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS atri_serial_driver crc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
