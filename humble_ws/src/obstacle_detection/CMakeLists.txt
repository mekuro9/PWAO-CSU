cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Create a library for LidarUtility
add_library(lidar_utility SHARED
  src/LidarUtility.cpp
)
target_include_directories(lidar_utility PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Create an executable
add_executable(obstacle_detector src/obstacle_detector.cpp)

# Declare dependencies
ament_target_dependencies(obstacle_detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Link the executable against the library
target_link_libraries(obstacle_detector
  lidar_utility
)

target_include_directories(obstacle_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the library and executable
install(TARGETS
  lidar_utility
  obstacle_detector
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

  
# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(lidar_utility)

ament_package()
