#!/usr/bin/env python3
"""
trajectory_planner.launch.py: MoveIt2와 trajectory planner를 함께 실행합니다.

사용법:
    # 기본 실행 (MoveIt + planner 대기)
    ros2 launch soarm101_trajectory_planner trajectory_planner.launch.py
    
    # RViz 없이 실행
    ros2 launch soarm101_trajectory_planner trajectory_planner.launch.py use_rviz:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz 실행 여부'
    )
    
    planner_arg = DeclareLaunchArgument(
        'planner',
        default_value='PTP',
        description='PILZ planner 타입 (PTP, LIN, CIRC)'
    )
    
    velocity_scale_arg = DeclareLaunchArgument(
        'velocity_scaling',
        default_value='0.1',
        description='속도 스케일링 (0.0~1.0)'
    )
    
    # MoveIt 설정 빌드
    moveit_config = (
        MoveItConfigsBuilder("soarm101_40mmUP", package_name="arm_moveit_config")
        .robot_description(file_path="config/soarm101_40mmUP.urdf.xacro")
        .robot_description_semantic(file_path="config/soarm101_40mmUP.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    
    # Move Group 노드
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[
            {"source_list": ["arm_trajectory_controller/joint_states"]},
        ],
    )
    
    # RViz (조건부)
    rviz_config = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    
    # ros2_control (Fake/Mock Hardware)
    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("arm_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    ])
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )
    
    # Controller spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_trajectory_controller", "-c", "/controller_manager"],
    )
    
    return LaunchDescription([
        # Arguments
        use_rviz_arg,
        planner_arg,
        velocity_scale_arg,
        
        # Nodes
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
