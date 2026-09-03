"""
real.launch.py — MoveIt2로 실제 SO-ARM101을 직접 구동.

mock ros2_control 대신 moveit_motor_bridge 노드를 띄워서, RViz의 Plan & Execute가
실제 모터를 바로 움직이게 한다. 한 줄로:

    ros2 launch soarm101_moveit_driver real.launch.py port:=/dev/ttyACM0

로봇 없이 RViz 파이프라인만 확인:

    ros2 launch soarm101_moveit_driver real.launch.py dry_run:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    port = LaunchConfiguration("port")
    robot_id = LaunchConfiguration("robot_id")
    dry_run = LaunchConfiguration("dry_run")
    bridge_python = LaunchConfiguration("bridge_python")

    moveit_config = (
        MoveItConfigsBuilder("soarm101_40mmUP", package_name="arm_moveit_config")
        .to_moveit_configs()
    )

    params_file = os.path.join(
        get_package_share_directory("soarm101_moveit_driver"), "config", "driver_params.yaml"
    )
    rviz_config = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "moveit.rviz"
    )

    # move_group — 실제 플래닝 (simple controller manager가 브리지의 액션 서버에 연결)
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # robot_state_publisher — URDF -> TF
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # RViz (MoveIt 플러그인)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # world -> base_link 고정 TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # 실제 모터 구동 브리지 (액션 서버 + /joint_states)
    #
    # lerobot이 별도 venv에만 설치된 경우, bridge_python:=<venv>/bin/python 으로
    # 그 인터프리터를 prefix로 지정해 노드를 실행한다. (rclpy는 ROS의 PYTHONPATH로
    # 상속되고, lerobot은 venv site-packages에서 로드됨)
    bridge = Node(
        package="soarm101_moveit_driver",
        executable="moveit_motor_bridge",
        output="screen",
        prefix=bridge_python,
        parameters=[params_file, {"port": port, "robot_id": robot_id, "dry_run": dry_run}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",
                              description="Feetech 모터 시리얼 포트"),
        DeclareLaunchArgument("robot_id", default_value="my_follower_arm",
                              description="LeRobot 캘리브레이션 ID"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true면 실제 모터 없이 RViz 표시만"),
        DeclareLaunchArgument("bridge_python", default_value="",
                              description="브리지 노드를 실행할 python 인터프리터 "
                                          "(lerobot이 venv에만 있을 때 <venv>/bin/python 지정)"),
        static_tf,
        rsp,
        move_group,
        rviz,
        bridge,
    ])
