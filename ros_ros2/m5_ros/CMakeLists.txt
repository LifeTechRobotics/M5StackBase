cmake_minimum_required(VERSION 3.8)
project(m5_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
	${rclcpp_INCLUDE_DIRS}
#	${std_srvs_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
)


add_executable(m5base
  src/m5_ros.cpp
  src/m5base_controller.cpp
  src/motor_control.cpp
)
ament_target_dependencies(m5base rclcpp sensor_msgs)
install(TARGETS m5base DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(sensor_msgs REQUIRED)
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
