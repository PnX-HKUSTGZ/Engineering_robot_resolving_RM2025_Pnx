cmake_minimum_required(VERSION 3.8)
project(position_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



include_directories("/opt/ros/humble/include/**")
include_directories("/usr/include/pcl-1.12/")
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/opencv4/")
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../..)  # Add project root directory

# find dependencies
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(pcl_conversions REQUIRED)


file(GLOB_RECURSE SOURCES "src/*.cpp")

message(${SOURCES})

add_library(PositionController SHARED ${SOURCES})

ament_target_dependencies(PositionController
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  Eigen3
  tf2_ros
  pcl_conversions
  tf2
  tf2_sensor_msgs
)

rclcpp_components_register_node(
  PositionController
  PLUGIN "Engineering_robot_RM2025_Pnx::PositionController"
  EXECUTABLE PositionController_node
)

install(TARGETS PositionController
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

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
