cmake_minimum_required(VERSION 3.8)
project(pointcloud_crop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(pcl_conversions REQUIRED)    # :contentReference[oaicite:0]{index=0}
find_package(pcl_ros         REQUIRED)    # ROS wrapper for PCL filters :contentReference[oaicite:1]{index=1}
find_package(PCL             REQUIRED COMPONENTS common filters)
find_package(yolo_msgs       REQUIRED) 
find_package(yaml-cpp REQUIRED)  
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_executable(pointcloud_crop src/pointcloud_crop.cpp)
target_include_directories(pointcloud_crop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pointcloud_crop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


ament_target_dependencies(pointcloud_crop
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  yolo_msgs
  yaml-cpp
)

target_link_libraries(pointcloud_crop
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS pointcloud_crop
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()