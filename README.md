# pilot-ros-bridge

Two-way bridge between PlatformPilot and ROS.

## Dependencies

neobotix-pilot-core >= 1.1.0 or neobotix-pilot-gtkgui >= 1.1.0

## Running

```
source ~/ros_workspace/devel/setup.bash
roslaunch pilot_ros_bridge mp_400.launch
roslaunch pilot_ros_bridge mpo_500.launch
roslaunch pilot_ros_bridge mpo_700.launch
```
