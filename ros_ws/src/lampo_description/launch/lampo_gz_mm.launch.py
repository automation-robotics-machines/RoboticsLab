import os
from os import environ
from os import pathsep
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.descriptions
from nav2_common.launch import RewrittenYaml
import yaml
import tempfile




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

    declared_arguments.append(
        DeclareLaunchArgument(
            "mm",
            default_value='false',
            description="mobile manipulators",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_namespace",
            default_value="r1_",
            description="Namespace and tf prefix for the robot"
        )
    )

    robot_namespace          = LaunchConfiguration("robot_namespace")
    description_package      = LaunchConfiguration("description_package")
    description_file         = LaunchConfiguration("description_file")
    mm                       = LaunchConfiguration("mm")



    robot_description_content_1 = launch_ros.descriptions.ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ","name:=",robot_namespace,"/mm1",
            " ","omni:=","true",
            " ","mm:=",mm,
            " ","prefix:=",robot_namespace,
            ]), value_type=str)

  

    robot_description_1  = {"robot_description": robot_description_content_1}
    frame_prefix_param_1 = {"frame_prefix": ""}
    
    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # name="robot_state_publisher_" + robot_namespace,
        namespace=robot_namespace,
        output="screen",
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ],
        parameters=[robot_description_1,frame_prefix_param_1,{"use_sim_time": True}],
    )

    spawn_sweepee_1 = Node(
        name='spawn1',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[ '-topic', [robot_namespace, '/robot_description'],
                   '-name', robot_namespace,
                   '-allow_renaming', 'true',
                   '-x', '-3.5',
                   '-y', '2.2',
                   '-z', '0.2',
                   '-Y', '0.3'],
        # remappings=[('/sweepee', [robot_namespace, '/robot_description'])],
        parameters=[{"use_sim_time": True}],
    )

########## TF merge #########

    tf_1 = Node(
        package="topic_tools",
        executable="relay",
        namespace=robot_namespace,
        name='relay_tf1_to_global',
        arguments=[['tf'], '/tf'],

    )

    tf_1s = Node(
        package="topic_tools",
        executable="relay",
        namespace=robot_namespace,
        name='relay_tf1s_to_global',
        arguments=[['tf_static'], '/tf_static'],

    )

    twist_repub = Node(
        package="lampo_description",
        executable="twist_repub.py",
        namespace=robot_namespace,
        name='twist_repub',
        parameters=[{'robot_namespace': robot_namespace}]
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

    def create_bridge_config(context):
        namespace = context.perform_substitution(robot_namespace)

        # Load original bridge config
        bridge_file = os.path.join(get_package_share_directory('lampo_description'), 'config', 'bridge.yaml')
        with open(bridge_file, 'r') as f:
            bridge_data = yaml.safe_load(f)

        # Perform substitutions
        for bridge_item in bridge_data:
            if 'ros_topic_name' in bridge_item:
                topic = bridge_item['ros_topic_name']
                if topic == '/prefix/cmd_vel':
                    bridge_item['ros_topic_name'] = f'/{namespace}/cmd_vel'
                elif topic == '/prefix/odom':
                    bridge_item['ros_topic_name'] = f'/{namespace}/odom'
                elif topic == '/prefix/lidar':
                    bridge_item['ros_topic_name'] = f'/{namespace}/lidar'
                elif topic == '/prefix/imu':
                    bridge_item['ros_topic_name'] = f'/{namespace}/imu'
                elif topic == '/prefix/tf':
                    bridge_item['ros_topic_name'] = f'/{namespace}/tf'

        # Write to temporary file
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(bridge_data, temp_file)
        temp_file.close()

        return [Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot_namespace,
            output="screen",
            parameters=[{"config_file": temp_file.name}, {"use_sim_time": True}],
        )]

    gz_bridge = OpaqueFunction(function=create_bridge_config)

########## UR CONTROLLERS 

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        namespace=robot_namespace,
        condition=IfCondition(mm),
    )

    start_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller"],
        namespace=robot_namespace,
        condition=IfCondition(mm),
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_position_controller"],
        namespace=robot_namespace,
        condition=IfCondition(mm),
    )

########## LAUNCHING


    nodes_to_start = [
        gazebo_server,
        rviz_node,
        tf_1,
        tf_1s,
        twist_repub,
        robot_state_publisher_node_1,
        TimerAction(
            period=5.0,
            actions=[spawn_sweepee_1,joint_state_broadcaster_spawner,start_controller_spawner,gripper_controller_spawner],
        ),
        TimerAction(
            period=1.0,
            actions=[gz_bridge]
        ),
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


