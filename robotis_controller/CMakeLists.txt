cmake_minimum_required(VERSION 3.5)
project(robotis_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_controller_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(robotis_device REQUIRED)
find_package(robotis_framework_common REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/robotis_controller/robotis_controller.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp 
  rclcpp 
  std_msgs 
  sensor_msgs 
  robotis_controller_msgs 
  dynamixel_sdk 
  robotis_device 
  robotis_framework_common 
)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  std_msgs
  sensor_msgs
  robotis_controller_msgs
  dynamixel_sdk
  robotis_device
  robotis_framework_common
)

ament_package()
