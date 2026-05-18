cmake_minimum_required(VERSION 3.8)
project(my_octomap_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

add_executable(save_octomap_node src/save_octomap_node.cpp)

ament_target_dependencies(save_octomap_node
  rclcpp
  octomap_msgs
  octomap
)

target_link_libraries(save_octomap_node
  octomap
)


add_executable(load_octomap_node src/load_octomap_node.cpp)

ament_target_dependencies(load_octomap_node
  rclcpp
  octomap_msgs
  sensor_msgs
)

target_link_libraries(load_octomap_node
  octomap
)

add_executable(rrt_3d_planner src/rrt_3d_planner.cpp)

ament_target_dependencies(rrt_3d_planner
  rclcpp
  octomap_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

target_link_libraries(rrt_3d_planner
  octomap
)

add_executable(path_follower_node src/path_follower_node.cpp)

ament_target_dependencies(path_follower_node
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  save_octomap_node
  load_octomap_node
  rrt_3d_planner
  path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
