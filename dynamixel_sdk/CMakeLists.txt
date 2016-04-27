cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(dynamixel_sdk
  src/${PROJECT_NAME}/PacketHandler.cpp
  src/${PROJECT_NAME}/Protocol1PacketHandler.cpp
  src/${PROJECT_NAME}/Protocol2PacketHandler.cpp
  src/${PROJECT_NAME}/GroupSyncRead.cpp
  src/${PROJECT_NAME}/GroupSyncWrite.cpp
  src/${PROJECT_NAME}/GroupBulkRead.cpp
  src/${PROJECT_NAME}/GroupBulkWrite.cpp
  src/${PROJECT_NAME}/PortHandler.cpp
  src/${PROJECT_NAME}_linux/PortHandlerLinux.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(dynamixel_sdk
  ${catkin_LIBRARIES}
)

