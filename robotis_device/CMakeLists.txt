cmake_minimum_required(VERSION 3.5)
project(robotis_device)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/robotis_device/robot.cpp
  src/robotis_device/sensor.cpp
  src/robotis_device/dynamixel.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  dynamixel_sdk
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${dynamixel_sdk_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY devices
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp dynamixel_sdk)

ament_package()
