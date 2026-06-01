cmake_minimum_required(VERSION 3.8)
project(behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(inrof2026_koma_type REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(bt_lib SHARED
  src/path_waypoint_position.cpp
  src/path_goal_position.cpp
  src/pursuit.cpp
)
target_include_directories(bt_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(bt_lib
  rclcpp
  rclcpp_action
  behaviortree_cpp
  inrof2026_koma_type
)
install(TARGETS bt_lib DESTINATION lib/${PROJECT_NAME})

add_executable(bt src/bt.cpp)
ament_target_dependencies(bt
  rclcpp
  rclcpp_action
  inrof2026_koma_type
  behaviortree_cpp
  tf2_geometry_msgs
)
target_include_directories(bt
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
set_target_properties(bt PROPERTIES INSTALL_RPATH "$ORIGIN")
target_link_libraries(bt bt_lib)
install(TARGETS
  bt
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
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
