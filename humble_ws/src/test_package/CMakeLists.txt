cmake_minimum_required(VERSION 3.8)
project(test_package)

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
find_package(message_filters REQUIRED)

add_executable(rtt_test src/rtt_test.cpp)
add_executable(rtt_message src/rtt_message.cpp)
ament_target_dependencies(rtt_test rclcpp std_msgs)
ament_target_dependencies(rtt_message rclcpp std_msgs)

add_executable(is_test src/is_test.cpp)
add_executable(is_message src/is_message.cpp)
ament_target_dependencies(is_test rclcpp std_msgs)
ament_target_dependencies(is_message rclcpp std_msgs)

add_executable(test_sync src/test_sync.cpp)
ament_target_dependencies(test_sync rclcpp std_msgs message_filters)

target_compile_features(rtt_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(rtt_message PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(is_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(is_message PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(test_sync PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS rtt_test
  rtt_message
  is_test
  is_message
  test_sync
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
