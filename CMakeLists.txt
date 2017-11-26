cmake_minimum_required(VERSION 2.8.3)
project(turn_table)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
find_path(LIBUSB_INCLUDEDIR 
          NAMES libusb.h
          HINTS /usr/include/libusb-1.0)
find_library(LIBUSB_LIBRARIES
             NAMES usb-1.0
             HINTS /usr/lib/ /usr/x86_64-linux-gnu/
			 PATH_SUFFIXES lib)

################################################
## Declare ROS messages, services and actions ##
################################################
add_service_files(
  FILES
  MoveToAngle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${LIBUSB_INCLUDEDIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
 add_executable(turn_table_driver src/turn_table_driver.cpp src/ArcusPerformaxDriver.c)

## Specify libraries to link a library or executable target against
 target_link_libraries(turn_table_driver
   ${LIBUSB_LIBRARIES}
   ${catkin_LIBRARIES}
 )
