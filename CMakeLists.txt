cmake_minimum_required(VERSION 3.8)
project(tf2_lookup_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(tf2_lookup_server_node src/tf2_lookup_server.cpp)
target_link_libraries(tf2_lookup_server_node Eigen3::Eigen)
ament_target_dependencies(tf2_lookup_server_node rclcpp rclcpp_action tf2 tf2_ros tf2_msgs)
install(TARGETS tf2_lookup_server_node DESTINATION lib/${PROJECT_NAME})

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

ament_package()
