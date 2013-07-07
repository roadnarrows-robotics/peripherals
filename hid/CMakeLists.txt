cmake_minimum_required(VERSION 2.8.3)
project(hid)

find_package(catkin 
  REQUIRED COMPONENTS 
    message_generation 
    roscpp 
    rospy 
    std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_library(
rnr_LIBRARIES2
NAMES hid rnr
PATHS /prj/lib /prj/lib/rnr
)

#catkin_python_setup()

# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hid
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(include
  /prj/include
  /opt/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(xbox_360 src/xbox_360/xbox_360.cpp)

# add_dependencies(hid_node hid_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries( 
  xbox_360
  ${rnr_LIBRARIES2}
  ${catkin_LIBRARIES}
)
