cmake_minimum_required(VERSION 3.8)
project(uart_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(inrof2026_koma_type REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(motor src/motor.cpp)
ament_target_dependencies(motor rclcpp geometry_msgs std_msgs nav_msgs)
target_include_directories(motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  motor
  DESTINATION lib/${PROJECT_NAME})

add_executable(color src/color.cpp)
ament_target_dependencies(color rclcpp std_msgs inrof2026_koma_type)
target_include_directories(color PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  color
  DESTINATION lib/${PROJECT_NAME})

add_executable(dummy_color src/dummy_color.cpp)
ament_target_dependencies(dummy_color rclcpp std_msgs inrof2026_koma_type)
target_include_directories(dummy_color PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS
  dummy_color
  DESTINATION lib/${PROJECT_NAME})

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
