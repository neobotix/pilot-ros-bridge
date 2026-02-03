# Neobotix GmbH
# Author: Pradheep Padmanabhan
# Contributor: Adarsh Karan K P

import launch
import xacro
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  IncludeLaunchDescription,
  OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.launch_context import LaunchContext
from launch.conditions import UnlessCondition

def execution_stage(context: LaunchContext,
                    robot_type,
                    ):

    robot_typ = str(robot_type.perform(context))
    launch_actions = []

    if robot_typ.startswith("rox-"):
        robot_pkg = get_package_share_directory('rox_description')
        urdf = os.path.join(robot_pkg, 'urdf', 'rox.urdf.xacro')

        # Extract rox_type (e.g. rox-diff -> diff)
        try:
            rox_variant = robot_typ.split('-', 1)[1]
        except IndexError:
            # Fallback or error if format is wrong, though user promised format
             rox_variant = "argo"

        joint_type = "fixed"
        if rox_variant in ["diff", "trike"]:
            joint_type = "revolute"

        xacro_args = [
             "xacro", " ", urdf,
             " ", "rox_type:=", rox_variant,
             " ", "joint_type:=", joint_type
        ]

        pilot_config_param = "/home/neobotix/rox_workspace/src/pilot-ros-bridge/config/default/" + rox_variant + "/"

    else:
        robot_pkg = get_package_share_directory('neo_'+ robot_typ + '-2')

        # Setting up the URDF
        urdf = os.path.join(robot_pkg,
            'robot_model',
            robot_typ + '.urdf.xacro')

        xacro_args = [
            "xacro", " ", urdf
        ]

        pilot_config_param = "/home/neobotix/"+ robot_typ +"_workspace/src/pilot-ros-bridge/config/default/" + robot_typ + "/"

    # Start robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(xacro_args), value_type=str),
        }],
        arguments=[urdf]
    )

    launch_actions.append(start_robot_state_publisher_cmd)

    # ToDo: Fix it
    pilot_config = os.path.join(
                get_package_share_directory('pilot_ros_bridge'),
                'config/default/generic'
                )

    start_pilot_ros_bridge = Node(
        package='pilot_ros_bridge',
        executable='pilot_ros_bridge_node',
        name='pilot_ros_bridge',
        output='screen',
        parameters=[{
            # Switch the workspace here
            'pilot_config': pilot_config_param,
        }],
    )

    launch_actions.append(start_pilot_ros_bridge)

    return launch_actions

def generate_launch_description():

    # Declare the launch arguments
    declare_robot_type_cmd = DeclareLaunchArgument(
            'robot_type', default_value='mpo_700',
            description='robot type that we want the bridge to connect to'
        )

    opq_function = OpaqueFunction(
        function=execution_stage, 
        args=[
            LaunchConfiguration('robot_type'),
            ])

    ld = LaunchDescription([
        declare_robot_type_cmd,
        opq_function
    ])
    return ld
