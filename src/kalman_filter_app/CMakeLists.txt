cmake_minimum_required(VERSION 3.5)
project(kalman_filter_app)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kalman_filter_core REQUIRED)
find_package(motion_model REQUIRED)
find_package(observation_model REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(geometry_msgs  REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(common_utils REQUIRED)

add_executable(${PROJECT_NAME} src/EkfOdomImuNode.cpp src/SensorFusionNode.cpp src/SensorFusionImuOdom.cpp )
ament_target_dependencies(${PROJECT_NAME} kalman_filter_core motion_model observation_model Eigen3 std_msgs sensor_msgs geometry_msgs common_utils nav_msgs rclcpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS kalman_filter_app
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION  share/${PROJECT_NAME})

ament_package()
