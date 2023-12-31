cmake_minimum_required(VERSION 3.8)
project(gazebo_exercise)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Package dependencies
set(PACKAGE_DEPENDENCIES
  ament_cmake
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

foreach(dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_executable(walker_node src/walker_node.cpp)
ament_target_dependencies(walker_node ${PACKAGE_DEPENDENCIES})

target_include_directories(walker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_compile_features(walker_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS walker_node
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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
