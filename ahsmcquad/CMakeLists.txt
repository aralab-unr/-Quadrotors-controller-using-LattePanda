cmake_minimum_required(VERSION 3.5)
project(ahsmcquad)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)  # For MAVROS communication


# Add executables
add_executable(getdata src/getdata.cpp)
add_executable(hsmcposition src/hsmcposition.cpp)
add_executable(cascadedpositionpid src/cascadedpositionpid.cpp)

# Link dependencies for mavlink_node
ament_target_dependencies(getdata 
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2 
  tf2_geometry_msgs
  mavros_msgs
)


ament_target_dependencies(hsmcposition 
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2 
  tf2_geometry_msgs
  mavros_msgs
)

ament_target_dependencies(cascadedpositionpid 
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2 
  tf2_geometry_msgs
  mavros_msgs
)


target_include_directories(getdata PUBLIC
  ${LibSerial_INCLUDE_DIRS}
  /opt/ros/humble/include/mavlink/v1.0/all
  ${LibSerial_LIBRARIES}
)


target_include_directories(hsmcposition PUBLIC
  ${LibSerial_INCLUDE_DIRS}
  /opt/ros/humble/include/mavlink/v1.0/all
  ${LibSerial_LIBRARIES}
)

target_include_directories(cascadedpositionpid PUBLIC
  ${LibSerial_INCLUDE_DIRS}
  /opt/ros/humble/include/mavlink/v1.0/all
  ${LibSerial_LIBRARIES}
)


# Install the executables
install(TARGETS
  getdata
  hsmcposition
  cascadedpositionpid
  DESTINATION lib/${PROJECT_NAME}
)

# Package configuration
ament_package()
