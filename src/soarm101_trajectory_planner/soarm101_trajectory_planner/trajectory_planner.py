#!/usr/bin/env python3
"""
TrajectoryPlanner: MoveIt2 PILZ planner를 사용하여 목표 좌표로의 경로를 계획합니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory

from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

import numpy as np
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum


class PilzMotionType(Enum):
    """PILZ planner에서 지원하는 모션 타입"""
    PTP = "pilz_industrial_motion_planner::CommandPlanner"  # Point-to-Point
    LIN = "pilz_industrial_motion_planner::CommandPlanner"  # Linear motion
    CIRC = "pilz_industrial_motion_planner::CommandPlanner"  # Circular motion


@dataclass
class PlanningResult:
    """경로 계획 결과를 담는 데이터 클래스"""
    success: bool
    trajectory: Optional[RobotTrajectory] = None
    joint_trajectory: Optional[JointTrajectory] = None
    error_message: str = ""
    planning_time: float = 0.0
    start_state: Optional[dict] = None
    goal_pose: Optional[dict] = None


class TrajectoryPlanner:
    """
    MoveIt2 PILZ planner를 사용하여 SO-ARM101의 경로를 계획하는 클래스
    
    사용 예시:
        planner = TrajectoryPlanner()
        result = planner.plan_to_position(x=0.2, y=0.1, z=0.15)
        if result.success:
            print(result.joint_trajectory)
    """
    
    def __init__(
        self,
        node_name: str = "trajectory_planner",
        planning_group: str = "arm",
        base_frame: str = "base_link",
        end_effector_link: str = "gripper_link",
        planner_id: str = "PTP",  # PTP, LIN, CIRC
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
    ):
        """
        TrajectoryPlanner 초기화
        
        Args:
            node_name: ROS2 노드 이름
            planning_group: MoveIt planning group 이름 (SRDF에서 정의)
            base_frame: 로봇의 베이스 프레임
            end_effector_link: 엔드 이펙터 링크 이름
            planner_id: PILZ planner 타입 (PTP, LIN, CIRC)
            velocity_scaling: 속도 스케일링 팩터 (0.0 ~ 1.0)
            acceleration_scaling: 가속도 스케일링 팩터 (0.0 ~ 1.0)
        """
        self.planning_group = planning_group
        self.base_frame = base_frame
        self.end_effector_link = end_effector_link
        self.planner_id = planner_id
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        
        self._logger = get_logger(node_name)
        self._moveit: Optional[MoveItPy] = None
        self._planning_component = None
        
    def initialize(self) -> bool:
        """
        MoveIt2 초기화. ROS2 context가 이미 초기화되어 있어야 함.
        
        Returns:
            초기화 성공 여부
        """
        try:
            self._logger.info("MoveIt2 초기화 중...")
            
            # MoveItPy 인스턴스 생성
            self._moveit = MoveItPy(node_name="trajectory_planner_moveit")
            
            # Planning component 가져오기
            self._planning_component = self._moveit.get_planning_component(self.planning_group)
            
            # PILZ planner 설정
            self._planning_component.set_planner_id(self.planner_id)
            
            self._logger.info(f"MoveIt2 초기화 완료. Planning group: {self.planning_group}")
            return True
            
        except Exception as e:
            self._logger.error(f"MoveIt2 초기화 실패: {e}")
            return False
    
    def set_planner_id(self, planner_id: str) -> None:
        """
        PILZ planner 타입 설정
        
        Args:
            planner_id: "PTP", "LIN", "CIRC" 중 하나
        """
        self.planner_id = planner_id
        if self._planning_component:
            self._planning_component.set_planner_id(planner_id)
            self._logger.info(f"Planner ID 설정: {planner_id}")
    
    def set_scaling_factors(
        self, 
        velocity: Optional[float] = None, 
        acceleration: Optional[float] = None
    ) -> None:
        """
        속도 및 가속도 스케일링 팩터 설정
        
        Args:
            velocity: 속도 스케일링 (0.0 ~ 1.0)
            acceleration: 가속도 스케일링 (0.0 ~ 1.0)
        """
        if velocity is not None:
            self.velocity_scaling = np.clip(velocity, 0.0, 1.0)
        if acceleration is not None:
            self.acceleration_scaling = np.clip(acceleration, 0.0, 1.0)
    
    def plan_to_position(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        start_joint_positions: Optional[dict] = None,
    ) -> PlanningResult:
        """
        목표 위치로의 경로를 계획합니다.
        
        Args:
            x, y, z: 목표 위치 (미터)
            roll, pitch, yaw: 목표 방향 (라디안)
            start_joint_positions: 시작 관절 위치 (None이면 현재 위치 사용)
            
        Returns:
            PlanningResult: 계획 결과
        """
        if self._planning_component is None:
            return PlanningResult(
                success=False,
                error_message="Planner가 초기화되지 않았습니다. initialize()를 먼저 호출하세요."
            )
        
        try:
            # 목표 Pose 생성
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.base_frame
            target_pose.pose.position.x = x
            target_pose.pose.position.y = y
            target_pose.pose.position.z = z
            
            # RPY를 Quaternion으로 변환
            quat = self._rpy_to_quaternion(roll, pitch, yaw)
            target_pose.pose.orientation.x = quat[0]
            target_pose.pose.orientation.y = quat[1]
            target_pose.pose.orientation.z = quat[2]
            target_pose.pose.orientation.w = quat[3]
            
            # 시작 상태 설정
            if start_joint_positions:
                self._planning_component.set_start_state_to_current_state()
                robot_state = self._moveit.get_robot_model().get_default_robot_state()
                for joint_name, position in start_joint_positions.items():
                    robot_state.set_joint_positions(joint_name, [position])
                self._planning_component.set_start_state(robot_state)
            else:
                self._planning_component.set_start_state_to_current_state()
            
            # 목표 설정
            self._planning_component.set_goal_state(
                pose_stamped_msg=target_pose,
                pose_link=self.end_effector_link
            )
            
            # 경로 계획
            self._logger.info(f"경로 계획 중... 목표: ({x:.3f}, {y:.3f}, {z:.3f})")
            
            plan_result = self._planning_component.plan()
            
            if plan_result:
                trajectory = plan_result.trajectory
                joint_trajectory = trajectory.joint_trajectory if trajectory else None
                
                # 속도/가속도 스케일링 적용
                if joint_trajectory:
                    joint_trajectory = self._apply_time_scaling(joint_trajectory)
                
                self._logger.info("경로 계획 성공!")
                
                return PlanningResult(
                    success=True,
                    trajectory=trajectory,
                    joint_trajectory=joint_trajectory,
                    planning_time=plan_result.planning_time if hasattr(plan_result, 'planning_time') else 0.0,
                    goal_pose={
                        'position': {'x': x, 'y': y, 'z': z},
                        'orientation': {'roll': roll, 'pitch': pitch, 'yaw': yaw}
                    }
                )
            else:
                return PlanningResult(
                    success=False,
                    error_message="경로 계획 실패: 유효한 경로를 찾을 수 없습니다."
                )
                
        except Exception as e:
            self._logger.error(f"경로 계획 중 오류: {e}")
            return PlanningResult(
                success=False,
                error_message=str(e)
            )
    
    def plan_to_joint_positions(
        self,
        joint_positions: dict,
    ) -> PlanningResult:
        """
        목표 관절 위치로의 경로를 계획합니다.
        
        Args:
            joint_positions: 목표 관절 위치 딕셔너리
                예: {'shoulder_pan': 0.0, 'shoulder_lift': 0.5, ...}
            
        Returns:
            PlanningResult: 계획 결과
        """
        if self._planning_component is None:
            return PlanningResult(
                success=False,
                error_message="Planner가 초기화되지 않았습니다."
            )
        
        try:
            self._planning_component.set_start_state_to_current_state()
            
            # 목표 관절 위치 설정
            self._planning_component.set_goal_state(
                configuration_name=None,
                joint_configuration=joint_positions
            )
            
            plan_result = self._planning_component.plan()
            
            if plan_result:
                trajectory = plan_result.trajectory
                joint_trajectory = trajectory.joint_trajectory if trajectory else None
                
                if joint_trajectory:
                    joint_trajectory = self._apply_time_scaling(joint_trajectory)
                
                return PlanningResult(
                    success=True,
                    trajectory=trajectory,
                    joint_trajectory=joint_trajectory,
                )
            else:
                return PlanningResult(
                    success=False,
                    error_message="관절 목표로의 경로 계획 실패"
                )
                
        except Exception as e:
            return PlanningResult(
                success=False,
                error_message=str(e)
            )
    
    def plan_to_named_target(self, target_name: str) -> PlanningResult:
        """
        SRDF에 정의된 named target으로 경로를 계획합니다.
        
        Args:
            target_name: SRDF에 정의된 목표 이름 (예: "home", "zero")
            
        Returns:
            PlanningResult: 계획 결과
        """
        if self._planning_component is None:
            return PlanningResult(
                success=False,
                error_message="Planner가 초기화되지 않았습니다."
            )
        
        try:
            self._planning_component.set_start_state_to_current_state()
            self._planning_component.set_goal_state(configuration_name=target_name)
            
            plan_result = self._planning_component.plan()
            
            if plan_result:
                trajectory = plan_result.trajectory
                joint_trajectory = trajectory.joint_trajectory if trajectory else None
                
                if joint_trajectory:
                    joint_trajectory = self._apply_time_scaling(joint_trajectory)
                
                return PlanningResult(
                    success=True,
                    trajectory=trajectory,
                    joint_trajectory=joint_trajectory,
                )
            else:
                return PlanningResult(
                    success=False,
                    error_message=f"Named target '{target_name}'으로의 경로 계획 실패"
                )
                
        except Exception as e:
            return PlanningResult(
                success=False,
                error_message=str(e)
            )
    
    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
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
    
    def _apply_time_scaling(self, trajectory: JointTrajectory) -> JointTrajectory:
        """
        속도/가속도 스케일링을 적용합니다.
        """
        # 간단한 시간 스케일링 (실제로는 time_optimal_trajectory_generation 사용 권장)
        scale_factor = 1.0 / max(self.velocity_scaling, 0.01)
        
        for point in trajectory.points:
            # 시간 스케일링
            original_secs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            scaled_secs = original_secs * scale_factor
            point.time_from_start.sec = int(scaled_secs)
            point.time_from_start.nanosec = int((scaled_secs % 1) * 1e9)
            
            # 속도 스케일링
            point.velocities = [v / scale_factor for v in point.velocities]
            
            # 가속도 스케일링
            if point.accelerations:
                point.accelerations = [a / (scale_factor ** 2) for a in point.accelerations]
        
        return trajectory
    
    def shutdown(self) -> None:
        """리소스 정리"""
        self._planning_component = None
        self._moveit = None
        self._logger.info("TrajectoryPlanner 종료")
