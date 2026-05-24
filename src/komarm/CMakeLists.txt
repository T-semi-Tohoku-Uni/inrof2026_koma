cmake_minimum_required(VERSION 3.8)
project(komarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# @libtorch
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/libtorch")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Torch REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(inrof2026_koma_type REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(feetech_servo REQUIRED)

add_executable(catch_influence src/catch_influence.cpp)
target_include_directories(catch_influence PUBLIC include)
ament_target_dependencies(catch_influence 
  rclcpp 
  sensor_msgs 
  geometry_msgs
  inrof2026_koma_type  
)
target_link_libraries(catch_influence "${TORCH_LIBRARIES}")
target_compile_options(catch_influence PRIVATE "${TORCH_CXX_FLAGS}")


install(TARGETS
  catch_influence
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(dummy_joint src/dummy_joint.cpp)
ament_target_dependencies(dummy_joint 
  rclcpp 
  sensor_msgs
  tf2_ros
)
target_include_directories(dummy_joint
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
install(TARGETS
  dummy_joint
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(feetech_joint src/feetech_joint.cpp)
ament_target_dependencies(feetech_joint 
  rclcpp 
  sensor_msgs
  feetech_servo
)
target_include_directories(feetech_joint
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
install(TARGETS
  feetech_joint
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(hand_pose src/hand_pose.cpp)
ament_target_dependencies(hand_pose 
  rclcpp 
  tf2_ros
  geometry_msgs
  inrof2026_koma_type
  visualization_msgs
)
target_include_directories(hand_pose
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
install(TARGETS
  hand_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf meshes include libtorch weight
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
