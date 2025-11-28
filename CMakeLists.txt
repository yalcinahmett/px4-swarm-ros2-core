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
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Advanced Swarm Graph Formation
add_executable(advanced_swarm src/advanced_swarm.cpp
src/utils/graph_math.cpp)
ament_target_dependencies(advanced_swarm
  "rclcpp"
  "px4_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "builtin_interfaces"
  "Eigen3"
)
target_link_libraries(advanced_swarm Eigen3::Eigen)
target_include_directories(advanced_swarm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(advanced_swarm PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17 

install(TARGETS advanced_swarm
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
