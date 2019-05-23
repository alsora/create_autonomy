cmake_minimum_required(VERSION 3.5)

project(ca_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(create_cmake_module REQUIRED)
find_package(libcreate REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ca_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  libcreate
  ca_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  include
  ${libcreate_INCLUDE_DIRS}
)

set(LIBRARY_NAME ${PROJECT_NAME})
add_library(${LIBRARY_NAME} SHARED src/create_driver_node.cpp)
ament_target_dependencies(${LIBRARY_NAME} ${dependencies})

add_executable(create_driver_main src/create_driver_main.cpp)
target_link_libraries(create_driver_main ${LIBRARY_NAME})
ament_target_dependencies(create_driver_main ${dependencies})

install(TARGETS create_driver_main ${LIBRARY_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${LIBRARY_DEPENDENCIES})

ament_package()