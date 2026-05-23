cmake_minimum_required(VERSION 3.8)
project(feetech_servo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(feetech_servo
  src/sts3125.cpp
)

target_include_directories(feetech_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(feetech_servo
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS feetech_servo
  EXPORT export_feetech_servo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_feetech_servo HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

ament_package()
