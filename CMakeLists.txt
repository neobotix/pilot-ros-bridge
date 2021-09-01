cmake_minimum_required(VERSION 2.8.3)
project(pilot_ros_bridge)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
	 nav_msgs
	 neo_msgs
	 neo_srvs
	 roscpp
	 tf
	 std_msgs
	 sensor_msgs
	 trajectory_msgs
 )

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  DIRECTORY msg
#  FILES EmergencyStopState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES std_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs tf nav_msgs sensor_msgs trajectory_msgs
    INCLUDE_DIRS generated/include
)

###########
## Build ##
###########

file(GLOB GENERATED_SOURCE_FILES generated/src/*.cpp)

add_library(pilot_ros_bridge SHARED
	${GENERATED_SOURCE_FILES}
)

include_directories(
	generated/include
	/opt/neobotix/pilot-core/include
	/opt/neobotix/pilot-gtkgui/include
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

link_directories(
	/opt/neobotix/pilot-core/lib
	/opt/neobotix/pilot-gtkgui/lib
)

add_executable(pilot_ros_bridge_node src/pilot_ros_bridge_node.cpp)

add_dependencies(pilot_ros_bridge_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(pilot_ros_bridge_node pilot_ros_bridge pilot_kinematics_differential pilot_kinematics pilot_core vnx_base ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pilot_ros_bridge pilot_ros_bridge_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
