cmake_minimum_required(VERSION 2.8.3)
project(imu)

find_package(catkin 
  REQUIRED COMPONENTS 
    std_msgs
    message_generation 
    roscpp 
    rospy 
)

find_package(Boost REQUIRED COMPONENTS system)

find_library(
  rnr_LIBRARIES4
  NAMES rnr_imu rnr
  PATHS /prj/lib /prj/lib/rnr
)

#catkin_python_setup()

# add_message_files(
# DIRECTORY msg
# FILES
# )

# add_service_files(
# DIRECTORY srv
# FILES
# )

# generate_messages(
# DEPENDENCIES
# std_msgs
# )

catkin_package()

# include_directories(include
# /prj/include
# /opt/include
# ${catkin_INCLUDE_DIRS}
# ${Boost_INCLUDE_DIRS}
# )

# add_executable(xbox_360 src/xbox_360/xbox_360.cpp)

# target_link_libraries( 
# xbox_360
# ${rnr_LIBRARIES4}
# ${catkin_LIBRARIES}
# )

# add_dependencies(xbox_360 hid_generate_messages_cpp)
