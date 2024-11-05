cmake_minimum_required(VERSION 3.5)
project(robotis_framework_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotis_device REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${robotis_device_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/motion_module.h
  include/${PROJECT_NAME}/sensor_module.h
  include/${PROJECT_NAME}/singleton.h
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  robotis_device
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
