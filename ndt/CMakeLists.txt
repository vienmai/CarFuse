cmake_minimum_required(VERSION 3.5)

project(ndt_locator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)

set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/ndt.cpp
  src/map_publisher.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(ndt_node src/ndt_node.cpp)
target_include_directories(ndt_node
  PRIVATE include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(ndt_node ${dependencies})

target_link_libraries(ndt_node ${PROJECT_NAME} ${PCL_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  ndt_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()