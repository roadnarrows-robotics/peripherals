cmake_minimum_required(VERSION 2.8.3)
project(hid)

find_package(catkin 
  REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

# TODO DHP - create rnrpkgConfig.cmake for cmake compatibility with rnr libs
find_library(
  rnr_LIBRARIES
  NAMES hekateros Dynamixel botsense netmsgs rnr_appkit rnr
  PATHS /prj/lib /prj/lib/rnr
)

# catkin_python_setup()

#add_message_files(
#DIRECTORY msg
#FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

include_directories(
  include
  /prj/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(xbox_360 src/xbox_360/xbox_360.cpp)
target_link_libraries(
  xbox_360
  ${rnr_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(xbox_360 hid_generate_messages_cpp)

