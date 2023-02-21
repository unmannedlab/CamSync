cmake_minimum_required(VERSION 2.8.3)
# Compile options so compiles with ROS Kinetic
# Future releases may want to 
add_compile_options(-std=c++11)
project(syncverify)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    message_generation
)

add_message_files(
    FILES
    SyncVerify.msg
    Ins.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vectornav
   CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
 #  DEPENDS system_lib
)
