cmake_minimum_required(VERSION 3.8)
project(myagv_ros2_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

# Update the executable definition to include both source files
add_executable(myagv_odom 
    src/main.cpp
    src/myagv_odom_node.cpp
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)


target_include_directories(myagv_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(myagv_odom PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  myagv_odom
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
  "tf2"
)

target_link_libraries(myagv_odom
  Boost::system
  Eigen3::Eigen
)

install(TARGETS myagv_odom
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/
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
