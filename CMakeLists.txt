cmake_minimum_required(VERSION 3.5)
project(pilot_ros_bridge)

set(CMAKE_CXX_STANDARD 14)

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
find_package(visualization_msgs REQUIRED)
find_package(std_srvs)

set(dependencies
	geometry_msgs
	std_srvs
	nav_msgs
	neo_msgs2
	neo_srvs2
	trajectory_msgs
	sensor_msgs
	tf2_geometry_msgs
	rclcpp
	tf2_ros
	visualization_msgs
)

ament_export_include_directories(
	include
	generated/include
)

###########
## Build ##
###########

link_directories(
	/opt/neobotix/pilot-core/lib
	/opt/neobotix/pilot-gtkgui/lib
)

file(GLOB GENERATED_SOURCE_FILES generated/src/*.cpp)

add_library(pilot_ros_bridge SHARED
	${GENERATED_SOURCE_FILES}
	src/ROS_Bridge.cpp
)

target_include_directories(pilot_ros_bridge PUBLIC
	include
	generated/include
	/opt/neobotix/pilot-core/include
	/opt/neobotix/pilot-gtkgui/include
)

target_link_libraries(pilot_ros_bridge
	pilot_kinematics_bicycle
	pilot_kinematics_differential
	pilot_kinematics_mecanum
	pilot_kinematics_omnidrive
	pilot_kinematics
	pilot_core
	vnx_base
)

add_executable(pilot_ros_bridge_node
	src/pilot_ros_bridge_node.cpp
)

ament_target_dependencies(pilot_ros_bridge ${dependencies})

ament_export_dependencies(pilot_ros_bridge geometry_msgs
	nav_msgs
	sensor_msgs
	std_msgs
	trajectory_msgs
	visualization_msgs
	rclcpp
	tf2_ros
)

target_link_libraries(pilot_ros_bridge_node
	pilot_ros_bridge
)

install(TARGETS pilot_ros_bridge DESTINATION lib/${PROJECT_NAME})
install(TARGETS pilot_ros_bridge_node DESTINATION bin/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_libraries(pilot_ros_bridge)

ament_package()

