cmake_minimum_required(VERSION 3.10)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(yaml REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
