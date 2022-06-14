cmake_minimum_required(VERSION 3.5)
project(pilot-ros-bridge)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(neo_msgs2 REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(pilot_ros_bridge_node src/pilot_ros_bridge_node.cpp)

set(dependencies
	geometry_msgs
	nav_msgs
	neo_msgs2
	neo_srvs2
	trajectory_msgs
	sensor_msgs
  tf2_geometry_msgs
	rclcpp
  tf2_ros
	)

ament_target_dependencies(pilot_ros_bridge_node	 ${dependencies})

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
)

link_directories(
	/opt/neobotix/pilot-core/lib
	/opt/neobotix/pilot-gtkgui/lib
)

ament_export_include_directories(generated/include)

ament_export_dependencies(pilot_ros_bridge_node	 geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	trajectory_msgs
	rclcpp
  tf2_ros)


target_link_libraries(pilot_ros_bridge_node pilot_ros_bridge 
	/opt/neobotix/pilot-core/lib/libpilot_kinematics_differential.so
	/opt/neobotix/pilot-core/lib/libpilot_kinematics.so
	/opt/neobotix/pilot-core/lib/libpilot_core.so
	/opt/neobotix/pilot-core/lib/libvnx_base.so)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS pilot_ros_bridge_node pilot_ros_bridge
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY generated/include/
  DESTINATION include)

ament_package()
