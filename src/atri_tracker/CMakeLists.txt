cmake_minimum_required(VERSION 3.10)
project(atri_tracker)

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
find_package(std_msgs REQUIRED)
find_package(atri_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(angles REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

# ================= 创建共享库 =================
add_library(tracker_node SHARED src/tracker_node.cpp)

add_library(tracker SHARED src/tracker.cpp)

add_library(extended_kalman_filter SHARED src/extended_kalman_filter.cpp)

add_library(gauss_newton_solver SHARED src/gauss_newton_solver.cpp)

# ================= 设置头文件目录 =================
target_include_directories(
  tracker_node PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(
  tracker PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(
  extended_kalman_filter
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_include_directories(
  gauss_newton_solver
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# ================= 链接ros依赖 =================
ament_target_dependencies(
  tracker_node
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  atri_interfaces
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
  tf2_ros)

ament_target_dependencies(tracker tf2_geometry_msgs atri_interfaces
                          geometry_msgs angles)

# =============== 链接库 ================
target_link_libraries(tracker_node ${OpenCV_LIBS} Eigen3::Eigen tracker
                      extended_kalman_filter)

target_link_libraries(tracker Eigen3::Eigen extended_kalman_filter
                      gauss_newton_solver)

target_link_libraries(extended_kalman_filter Eigen3::Eigen)

target_link_libraries(gauss_newton_solver Eigen3::Eigen)

# ============== 注册组件节点 =================
rclcpp_components_register_node(tracker_node PLUGIN "atri_tracker::TrackerNode"
                                EXECUTABLE tracker_node_exe)

# ================ 安装 =================
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

install(
  TARGETS tracker_node tracker extended_kalman_filter gauss_newton_solver
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
