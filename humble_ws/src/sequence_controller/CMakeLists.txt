cmake_minimum_required(VERSION 3.8)
project(sequence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(latency_test src/latency_test.cpp)
ament_target_dependencies(latency_test rclcpp std_msgs)

add_executable(sequence src/sequence_controller.cpp)
ament_target_dependencies(sequence rclcpp std_msgs)

add_executable(state src/state_controller.cpp)
ament_target_dependencies(state rclcpp std_msgs geometry_msgs)

add_executable(state2 src/state_controller_v2.cpp)
ament_target_dependencies(state2 rclcpp std_msgs geometry_msgs)


target_include_directories(latency_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
target_compile_features(latency_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS latency_test
  sequence
  state
  state2
  DESTINATION lib/${PROJECT_NAME})

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
