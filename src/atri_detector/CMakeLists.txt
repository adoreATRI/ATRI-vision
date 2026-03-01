cmake_minimum_required(VERSION 3.8)
project(atri_detector)

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
find_package(tf2_geometry_msgs REQUIRED) 

find_package(tf2 REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED)

# ==================== ONNX Runtime ====================
set(ONNXRUNTIME_DIR $ENV{ONNXRUNTIME_DIR})
find_path(ONNXRUNTIME_INCLUDE_DIR "onnxruntime_cxx_api.h" HINTS ${ONNXRUNTIME_DIR}/include)
find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_DIR}/lib)
find_library(ONNXRUNTIME_CUDA_LIB onnxruntime_providers_cuda HINTS ${ONNXRUNTIME_DIR}/lib)
find_library(ONNXRUNTIME_SHARED_LIB onnxruntime_providers_shared HINTS ${ONNXRUNTIME_DIR}/lib)

# ================= 创建共享库 =================
add_library(detector_node SHARED
  src/detector_node.cpp
)

add_library(detector SHARED
  src/detector.cpp
  src/pnp_solver.cpp
  src/onnx_inference.cpp
)


# ================= 设置头文件目录 =================
target_include_directories(detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>
)
target_include_directories(detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${ONNXRUNTIME_INCLUDE_DIR}>
)


# ================= 链接ros依赖 =================
ament_target_dependencies(detector_node
rclcpp
rclcpp_components

sensor_msgs
std_msgs
atri_interfaces
tf2_geometry_msgs

tf2
)

ament_target_dependencies(detector
atri_interfaces

ament_index_cpp
)

# =============== 链接库 ================
target_link_libraries(detector_node
  detector
)

target_link_libraries(detector
  ${OpenCV_LIBS}
  ${ONNXRUNTIME_LIB}
  ${ONNXRUNTIME_CUDA_LIB}
  ${ONNXRUNTIME_SHARED_LIB}

  yaml-cpp
)


# ============== 注册组件节点 =================
rclcpp_components_register_node(detector_node
  PLUGIN "atri_detector::DetectorNode"
  EXECUTABLE detector_node_exe
)

# ================ 安装 =================
install(TARGETS detector detector_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config launch model
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()