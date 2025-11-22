cmake_minimum_required(VERSION 3.8)
project(px4-swarm-ros2-core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Listeners
add_executable(listener_node src/listener_node.cpp)
ament_target_dependencies(listener_node
  "rclcpp"
  "px4_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "builtin_interfaces"
)

target_include_directories(listener_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(listener_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS listener_node
  DESTINATION lib/${PROJECT_NAME})

# Offboard Control
add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control
  "rclcpp"
  "px4_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "builtin_interfaces"
)

target_include_directories(offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(offboard_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS offboard_control
  DESTINATION lib/${PROJECT_NAME})

# Basic Offboard Control
add_executable(basic_offboard_control src/basic_control.cpp)
ament_target_dependencies(basic_offboard_control
  "rclcpp"
  "px4_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "builtin_interfaces"
)

target_include_directories(basic_offboard_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(basic_offboard_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS basic_offboard_control
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
