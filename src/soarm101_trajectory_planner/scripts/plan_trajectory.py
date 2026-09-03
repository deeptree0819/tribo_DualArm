#!/usr/bin/env python3
"""
plan_trajectory.py: 목표 좌표를 입력받아 OMPL planner로 경로를 계획하고 YAML로 저장합니다.

주의: 이 스크립트를 실행하기 전에 MoveIt을 먼저 실행해야 합니다:
    ros2 launch arm_moveit_config demo.launch.py

사용 예시:
    ros2 run soarm101_trajectory_planner plan_trajectory.py --x 0.2 --y 0.1 --z 0.15
    ros2 run soarm101_trajectory_planner plan_trajectory.py --target home
"""

import argparse
import sys
import os
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    MotionPlanRequest, 
    PlanningOptions,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    RobotState as RobotStateMsg,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from soarm101_trajectory_planner.trajectory_saver import TrajectorySaver

import numpy as np
from threading import Event


class TrajectoryPlannerNode(Node):
    """MoveIt move_group에 연결하여 경로를 계획하는 노드"""
    
    def __init__(
        self,
        planning_group: str = "arm",
        planner_id: str = "RRTConnect",  # OMPL planner
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
    ):
        super().__init__('trajectory_planner_node')
        
        self.planning_group = planning_group
        self.planner_id = planner_id
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        
        # Planning parameters
        self.planning_time = 5.0
        self.num_planning_attempts = 10
        self.base_frame = "base_link"
        self.end_effector_link = "gripper_link"
        
        # Joint configuration
        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift", 
            "elbow_flex",
            "wrist_flex",
            "wrist_roll"
        ]
        
        # Named targets from SRDF
        self.named_targets = {
            "home": {
                "shoulder_pan": 0.0,
                "shoulder_lift": 0.9,
                "elbow_flex": 1.2,
                "wrist_flex": 1.1,
                "wrist_roll": 0.0
            },
            "zero": {
                "shoulder_pan": 0.0,
                "shoulder_lift": 0.0,
                "elbow_flex": 0.0,
                "wrist_flex": 0.0,
                "wrist_roll": 0.0
            }
        }
        
        # Current joint state
        self._current_joint_state = None
        
        # Callback group
        self._callback_group = ReentrantCallbackGroup()
        
        # Service clients
        self._plan_client = self.create_client(
            GetMotionPlan, 
            '/plan_kinematic_path',
            callback_group=self._callback_group
        )
        
        # Joint state subscriber
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self._callback_group
        )
        
        self.get_logger().info("TrajectoryPlannerNode 초기화 완료")
        
    def _joint_state_callback(self, msg: JointState):
        """Joint state 콜백"""
        self._current_joint_state = msg
    
    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """서비스 연결 대기"""
        self.get_logger().info("MoveIt 서비스 연결 대기 중...")
        
        if not self._plan_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                "'/plan_kinematic_path' 서비스에 연결할 수 없습니다. "
                "MoveIt이 실행 중인지 확인하세요: ros2 launch arm_moveit_config demo.launch.py"
            )
            return False
        
        self.get_logger().info("MoveIt 서비스 연결됨")
        return True
    
    def wait_for_joint_state(self, timeout_sec: float = 5.0) -> bool:
        """Joint state 수신 대기"""
        self.get_logger().info("Joint state 대기 중...")
        
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._current_joint_state is not None:
                self.get_logger().info("Joint state 수신됨")
                return True
        
        self.get_logger().warn("Joint state를 받지 못했습니다. 기본값 사용.")
        self._current_joint_state = JointState()
        self._current_joint_state.name = self.joint_names
        self._current_joint_state.position = [0.0] * len(self.joint_names)
        return True
    
    def get_current_joint_positions(self) -> dict:
        """현재 관절 위치 반환"""
        if self._current_joint_state is None:
            return {name: 0.0 for name in self.joint_names}
        
        positions = {}
        for i, name in enumerate(self._current_joint_state.name):
            if name in self.joint_names:
                positions[name] = self._current_joint_state.position[i]
        return positions
    
    def plan_to_joint_target(self, joint_positions: dict):
        """
        관절 목표로 경로 계획
        
        Args:
            joint_positions: {'joint_name': angle, ...}
            
        Returns:
            (success, trajectory, error_msg)
        """
        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        
        # OMPL planner 설정
        request.pipeline_id = "ompl"
        request.planner_id = self.planner_id
        
        # 시작 상태 설정
        if self._current_joint_state is not None:
            request.start_state.joint_state = self._current_joint_state
            request.start_state.is_diff = False
        
        # 목표 제약 조건 설정 (Joint constraints)
        goal_constraints = Constraints()
        
        for joint_name, position in joint_positions.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        request.goal_constraints.append(goal_constraints)
        
        return self._call_plan_service(request)
    
    def plan_to_pose(
        self,
        x: float, y: float, z: float,
        roll: float = 0.0, pitch: float = 1.57, yaw: float = 0.0,
        position_only: bool = False,
        orientation_tolerance: float = 3.14,
    ):
        """
        Pose 목표로 경로 계획
        
        Args:
            x, y, z: 목표 위치 (미터)
            roll, pitch, yaw: 목표 방향 (라디안). 기본값 pitch=1.57 (아래 방향)
            position_only: True이면 orientation 제약 없이 위치만 목표로 설정
            orientation_tolerance: orientation 허용 오차 (라디안). 5DOF 팔이므로 넉넉하게 설정
            
        Returns:
            (success, trajectory, error_msg)
        """
        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        
        # OMPL planner 설정
        request.pipeline_id = "ompl"
        request.planner_id = self.planner_id
        
        # 시작 상태
        if self._current_joint_state is not None:
            request.start_state.joint_state = self._current_joint_state
            request.start_state.is_diff = False
        
        # Position constraint
        goal_constraints = Constraints()
        
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.weight = 1.0
        
        # Target position as a small sphere
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.02]  # 20mm tolerance (기존 10mm에서 확대)
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        
        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose)
        
        position_constraint.constraint_region = bounding_volume
        goal_constraints.position_constraints.append(position_constraint)
        
        # Orientation constraint (5DOF 팔이므로 position_only 옵션 제공)
        if not position_only:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.weight = 1.0
            
            quat = self._rpy_to_quaternion(roll, pitch, yaw)
            orientation_constraint.orientation.x = quat[0]
            orientation_constraint.orientation.y = quat[1]
            orientation_constraint.orientation.z = quat[2]
            orientation_constraint.orientation.w = quat[3]
            orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
            
            goal_constraints.orientation_constraints.append(orientation_constraint)
            self.get_logger().info(
                f"Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}, "
                f"tolerance={orientation_tolerance:.2f} rad"
            )
        else:
            self.get_logger().info("Position-only 모드: orientation 제약 없음")
        
        request.goal_constraints.append(goal_constraints)
        
        return self._call_plan_service(request)
    
    def plan_to_named_target(self, target_name: str):
        """
        Named target으로 경로 계획
        
        Returns:
            (success, trajectory, error_msg)
        """
        if target_name not in self.named_targets:
            return False, None, f"Unknown target: {target_name}. Available: {list(self.named_targets.keys())}"
        
        joint_positions = self.named_targets[target_name]
        self.get_logger().info(f"Named target '{target_name}': {joint_positions}")
        
        return self.plan_to_joint_target(joint_positions)
    
    def _call_plan_service(self, request: MotionPlanRequest):
        """Planning 서비스 호출"""
        self.get_logger().info(f"경로 계획 요청 중... (pipeline: {request.pipeline_id}, planner: {request.planner_id})")
        
        srv_request = GetMotionPlan.Request()
        srv_request.motion_plan_request = request
        
        future = self._plan_client.call_async(srv_request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is None:
            return False, None, "Planning service call failed (timeout or error)"
        
        response = future.result()
        
        if response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
            self.get_logger().info(f"경로 계획 성공! Points: {len(trajectory.points)}")
            
            if trajectory.points:
                last_t = trajectory.points[-1].time_from_start
                duration = last_t.sec + last_t.nanosec * 1e-9
                self.get_logger().info(f"Trajectory duration: {duration:.3f}초")
            
            return True, trajectory, ""
        else:
            error_code = response.motion_plan_response.error_code.val
            error_msg = f"Planning failed with error code: {error_code}"
            self.get_logger().error(error_msg)
            return False, None, error_msg
    
    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float):
        """Roll, Pitch, Yaw를 Quaternion으로 변환"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)


def main():
    parser = argparse.ArgumentParser(
        description='SO-ARM101 OMPL Trajectory Planner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
주의: 이 스크립트 실행 전에 MoveIt을 먼저 실행하세요:
    ros2 launch arm_moveit_config demo.launch.py

사용 예시:
  # Named target 사용
  ros2 run soarm101_trajectory_planner plan_trajectory.py --target home
  ros2 run soarm101_trajectory_planner plan_trajectory.py --target zero
  
  # XYZ 좌표로 경로 계획
  ros2 run soarm101_trajectory_planner plan_trajectory.py --x 0.2 --y 0.1 --z 0.15
  
  # 방향까지 지정
  ros2 run soarm101_trajectory_planner plan_trajectory.py --x 0.2 --y 0.0 --z 0.2 --roll 0 --pitch 1.57 --yaw 0
  
  # 다른 OMPL planner 사용
  ros2 run soarm101_trajectory_planner plan_trajectory.py --target home --planner RRTstar
  
  # 속도 스케일 조정 (더 빠르게)
  ros2 run soarm101_trajectory_planner plan_trajectory.py --target home -v 0.5
  
  # 출력 파일 지정
  ros2 run soarm101_trajectory_planner plan_trajectory.py --target home -o my_trajectory.yaml
        """
    )
    
    # Position arguments
    parser.add_argument('--x', type=float, help='목표 X 좌표 (미터)')
    parser.add_argument('--y', type=float, help='목표 Y 좌표 (미터)')
    parser.add_argument('--z', type=float, help='목표 Z 좌표 (미터)')
    
    # Orientation arguments
    parser.add_argument('--roll', type=float, default=0.0, help='Roll (라디안)')
    parser.add_argument('--pitch', type=float, default=1.57, help='Pitch (라디안, 기본: 1.57 아래방향)')
    parser.add_argument('--yaw', type=float, default=0.0, help='Yaw (라디안)')
    
    # Orientation options
    parser.add_argument('--position-only', action='store_true',
                        help='Orientation 제약 없이 위치만 목표로 설정 (5DOF 팔 권장)')
    parser.add_argument('--orientation-tolerance', type=float, default=3.14,
                        help='Orientation 허용 오차 (라디안, 기본: 3.14 = 매우 느슨)')
    
    # Named target
    parser.add_argument('--target', '-t', type=str, help='Named target (home, zero)')
    
    # Joint positions (직접 지정)
    parser.add_argument('--joints', '-j', type=str, 
                        help='관절 위치 (예: "0.0,0.5,1.0,0.5,0.0")')
    
    # Planning parameters
    parser.add_argument('--planner', '-p', type=str, default='RRTConnect',
                        choices=['RRTConnect', 'RRTstar', 'PRM', 'PRMstar', 'LazyPRMstar', 'TRRT', 'EST', 'KPIECE', 'BKPIECE'],
                        help='OMPL planner 타입 (기본: RRTConnect)')
    parser.add_argument('--velocity-scale', '-v', type=float, default=0.1,
                        help='속도 스케일링 (0.0~1.0, 기본: 0.1)')
    parser.add_argument('--accel-scale', '-a', type=float, default=0.1,
                        help='가속도 스케일링 (0.0~1.0, 기본: 0.1)')
    parser.add_argument('--group', '-g', type=str, default='arm',
                        help='Planning group 이름 (기본: arm)')
    
    # Output
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='출력 YAML 파일 이름')
    parser.add_argument('--output-dir', '-d', type=str, default='./trajectories',
                        help='출력 디렉토리 (기본: ./trajectories)')
    parser.add_argument('--name', '-n', type=str, default='trajectory',
                        help='Trajectory 이름')
    
    args = parser.parse_args()
    
    # 입력 검증
    has_xyz = args.x is not None and args.y is not None and args.z is not None
    has_target = args.target is not None
    has_joints = args.joints is not None
    
    input_count = sum([has_xyz, has_target, has_joints])
    if input_count == 0:
        parser.error("--x/--y/--z, --target, 또는 --joints 중 하나를 지정해야 합니다.")
    if input_count > 1:
        parser.error("--x/--y/--z, --target, --joints 중 하나만 지정할 수 있습니다.")
    
    # ROS2 초기화
    rclpy.init()
    
    try:
        # 노드 생성
        node = TrajectoryPlannerNode(
            planning_group=args.group,
            planner_id=args.planner,
            velocity_scaling=args.velocity_scale,
            acceleration_scaling=args.accel_scale,
        )
        
        # 서비스 연결 대기
        if not node.wait_for_services(timeout_sec=10.0):
            print("\nERROR: MoveIt이 실행 중이지 않습니다.")
            print("먼저 다음 명령을 실행하세요:")
            print("  ros2 launch arm_moveit_config demo.launch.py")
            return 1
        
        # Joint state 대기
        node.wait_for_joint_state(timeout_sec=3.0)
        
        # 경로 계획
        goal_info = {}
        
        if has_target:
            success, trajectory, error = node.plan_to_named_target(args.target)
            goal_info = {'target_name': args.target}
            
        elif has_joints:
            joint_values = [float(v) for v in args.joints.split(',')]
            if len(joint_values) != 5:
                print("ERROR: --joints는 5개의 값이 필요합니다 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll)")
                return 1
            joint_positions = dict(zip(node.joint_names, joint_values))
            success, trajectory, error = node.plan_to_joint_target(joint_positions)
            goal_info = {'joint_positions': joint_positions}
            
        else:  # has_xyz
            success, trajectory, error = node.plan_to_pose(
                x=args.x, y=args.y, z=args.z,
                roll=args.roll, pitch=args.pitch, yaw=args.yaw,
                position_only=args.position_only,
                orientation_tolerance=args.orientation_tolerance,
            )
            goal_info = {
                'position': {'x': args.x, 'y': args.y, 'z': args.z},
                'orientation': {'roll': args.roll, 'pitch': args.pitch, 'yaw': args.yaw}
            }
        
        if not success:
            print(f"ERROR: 경로 계획 실패 - {error}")
            return 1
        
        # YAML로 저장
        saver = TrajectorySaver(output_dir=args.output_dir)
        
        filepath = saver.save(
            trajectory=trajectory,
            name=args.name,
            description=f"OMPL {args.planner} trajectory",
            goal_position=goal_info.get('position'),
            goal_orientation=goal_info.get('orientation'),
            planner_id=args.planner,
            planning_group=args.group,
            velocity_scaling=args.velocity_scale,
            acceleration_scaling=args.accel_scale,
            filename=args.output
        )
        
        print(f"\n✅ Trajectory 저장 완료!")
        print(f"   파일: {filepath}")
        print(f"   Points: {len(trajectory.points)}")
        if trajectory.points:
            last_time = trajectory.points[-1].time_from_start
            duration = last_time.sec + last_time.nanosec * 1e-9
            print(f"   Duration: {duration:.3f}초")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n중단됨")
        return 130
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
