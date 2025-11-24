import os
from os import environ
from os import pathsep
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from scripts import GazeboRosPaths
import launch_ros.descriptions

import xacro



def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="lampo_description",
            description="mobile manip description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="system.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    
    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")



    robot_description_content_1 = launch_ros.descriptions.ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","name:=","mm1",
            " ","omni:=","true",
            " ","prefix:=","sweepee_1/"
        ]), value_type=str)

    robot_description_content_2 = launch_ros.descriptions.ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","name:=","mm2",
            " ","omni:=","true",
            " ","prefix:=","sweepee_2/"
        ]), value_type=str)
    

    robot_description_1  = {"robot_description": robot_description_content_1}
    robot_description_2  = {"robot_description": robot_description_content_2}


    frame_prefix_param_1 = {"frame_prefix": ""}
    frame_prefix_param_2 = {"frame_prefix": ""}
    
    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sweepee_1",
        output="screen",
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[robot_description_1,frame_prefix_param_1,{"use_sim_time": True}],
    )

    robot_state_publisher_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sweepee_2",
        output="screen",
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[robot_description_2,frame_prefix_param_2,{"use_sim_time": True}],
    )

    spawn_sweepee_1 = Node(
        name='spawn1',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[ '-topic', 'sweepee_1/robot_description',
                   '-name', 'sweepee_1',
                   '-allow_renaming', 'true',
                   '-x', '-3.5',
                   '-y', '2.2',
                   '-z', '0.2',
                   '-Y', '0.3'],
        remappings=[('/sweepee', 'sweepee_1/robot_description')],
        parameters=[{"use_sim_time": True}],
    )

    spawn_sweepee_2 = Node(
        name='spawn2',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'sweepee_2/robot_description',
                   '-name', 'sweepee_2',
                   '-allow_renaming', 'true',
                   '-x', '-3.0',
                   '-y', '-2.0',
                   '-z', '0.2'],
        remappings=[('/sweepee', 'sweepee_2/robot_description')],
        parameters=[{"use_sim_time": True}],
    )

########## TF merge #########

    tf_1 = Node(
        package="topic_tools",
        executable="relay",
        name='relay_tf1_to_global',
        arguments=['sweepee_1/tf', '/tf'],

    )
    
    tf_1s = Node(
        package="topic_tools",
        executable="relay",
        name='relay_tf1s_to_global',
        arguments=['sweepee_1/tf_static', '/tf_static'],

    )

    twist_repub = Node(
        package="lampo_description",
        executable="twist_repub.py",
        name='twist_repub'

    )


########## VISUALIZATION

    world_path = os.path.join(get_package_share_directory('lampo_description'),'worlds/warehouse.sdf')

    gazebo_server = GroupAction(
        actions=[
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments={
                'gz_args': [' -r -v 4 ' + world_path ],
                'gz_version': "9"
            }.items())
            ]
            )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lampo_description"), "rviz", "config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_file],
    )


########## BRIDGE

    config_bridge = os.path.join(get_package_share_directory('lampo_description'),
                              'config', 'bridge.yaml')
    
    config_param  = {"config_file": config_bridge}

    gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen" ,
            parameters=[config_param,{"use_sim_time": True}],
        )

    # ros_to_gz_1 = Node(
    #         package="lampo_description",
    #         executable="ros_to_gz_commands.py",
    #         namespace="sweepee_1",
    #         output="screen",
    #         parameters=[{"use_sim_time": True}],
    #     )

    # ros_to_gz_2 = Node(
    #         package="lampo_description",
    #         executable="ros_to_gz_commands.py",
    #         namespace="sweepee_2",
    #         output="screen",
    #         parameters=[{"use_sim_time": True}],
    #     )
    
########## LAUNCHING


    nodes_to_start = [
        gazebo_server,
        rviz_node,
        tf_1,
        tf_1s,
        twist_repub,
        TimerAction(
            period=5.0,
            actions=[spawn_sweepee_1,robot_state_publisher_node_1],
        ),
        # TimerAction(
        #     period=4.0,
        #     actions=[spawn_sweepee_2,robot_state_publisher_node_2],
        # ),
        TimerAction(
            period=5.0,
            actions=[gz_bridge]
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


