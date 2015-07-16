cmake_minimum_required(VERSION 2.8.3)
project(mot)

find_package(catkin 
  REQUIRED COMPONENTS 
    std_msgs
    message_generation 
    roscpp 
    rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

find_library(
  rnr_LIBRARIES3
  NAMES mot rnr
  PATHS /prj/lib /prj/lib/rnr /usr/local/lib/ /usr/local/lib/rnr
)

#catkin_python_setup()

add_message_files(
DIRECTORY msg
FILES
  SpeedCmd.msg
)

add_service_files( DIRECTORY srv FILES SetSpeed.srv)

generate_messages( DEPENDENCIES std_msgs)

catkin_package()

include_directories(
  include
  /prj/include
  /opt/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#add_executable(roboteq_sm src/roboteq_sm/roboteq_sm.cpp)

#DHP -figure out why rnr_LIBS doesn't contain rnr
#target_link_libraries( 
#  roboteq_sm
#  ${rnr_LIBRARIES3} rnr
#  ${catkin_LIBRARIES}
#)

#add_dependencies(roboteq_sm mot_generate_messages_cpp)
