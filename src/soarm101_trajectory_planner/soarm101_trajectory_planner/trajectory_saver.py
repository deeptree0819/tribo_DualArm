#!/usr/bin/env python3
"""
TrajectorySaver: Joint Trajectory를 YAML 파일로 저장합니다.
"""

import yaml
import os
from datetime import datetime
from typing import Optional, List, Dict, Any
from dataclasses import dataclass, asdict

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


@dataclass
class TrajectoryPoint:
    """단일 trajectory point 데이터"""
    time_from_start: float  # seconds
    positions: List[float]
    velocities: List[float]
    accelerations: List[float]
    effort: List[float]


@dataclass
class TrajectoryData:
    """전체 trajectory 데이터"""
    # 메타데이터
    name: str
    created_at: str
    description: str
    
    # 목표 정보
    goal_position: Optional[Dict[str, float]] = None
    goal_orientation: Optional[Dict[str, float]] = None
    
    # Planning 정보
    planner_id: str = "PTP"
    planning_group: str = "arm"
    velocity_scaling: float = 0.1
    acceleration_scaling: float = 0.1
    
    # Joint 정보
    joint_names: List[str] = None
    
    # Trajectory points
    points: List[TrajectoryPoint] = None
    
    # 통계 정보
    num_points: int = 0
    total_duration: float = 0.0


class TrajectorySaver:
    """
    Joint Trajectory를 YAML 파일로 저장하는 클래스
    
    사용 예시:
        saver = TrajectorySaver(output_dir="./trajectories")
        saver.save(joint_trajectory, name="pick_motion", goal_xyz=(0.2, 0.1, 0.15))
    """
    
    def __init__(
        self,
        output_dir: str = "./trajectories",
        create_dir: bool = True
    ):
        """
        TrajectorySaver 초기화
        
        Args:
            output_dir: YAML 파일을 저장할 디렉토리
            create_dir: 디렉토리가 없을 경우 생성 여부
        """
        self.output_dir = output_dir
        
        if create_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
    
    def save(
        self,
        trajectory: JointTrajectory,
        name: str = "trajectory",
        description: str = "",
        goal_position: Optional[Dict[str, float]] = None,
        goal_orientation: Optional[Dict[str, float]] = None,
        planner_id: str = "PTP",
        planning_group: str = "arm",
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
        filename: Optional[str] = None,
    ) -> str:
        """
        Joint Trajectory를 YAML 파일로 저장
        
        Args:
            trajectory: JointTrajectory 메시지
            name: trajectory 이름
            description: 설명
            goal_position: 목표 위치 {'x': ..., 'y': ..., 'z': ...}
            goal_orientation: 목표 방향 {'roll': ..., 'pitch': ..., 'yaw': ...}
            planner_id: 사용된 planner ID
            planning_group: MoveIt planning group 이름
            velocity_scaling: 속도 스케일링 팩터
            acceleration_scaling: 가속도 스케일링 팩터
            filename: 저장할 파일 이름 (None이면 자동 생성)
            
        Returns:
            저장된 파일 경로
        """
        # Trajectory points 변환
        points = []
        for point in trajectory.points:
            time_secs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            
            trajectory_point = TrajectoryPoint(
                time_from_start=round(time_secs, 6),
                positions=[round(p, 6) for p in point.positions],
                velocities=[round(v, 6) for v in point.velocities] if point.velocities else [],
                accelerations=[round(a, 6) for a in point.accelerations] if point.accelerations else [],
                effort=[round(e, 6) for e in point.effort] if point.effort else []
            )
            points.append(trajectory_point)
        
        # 총 duration 계산
        total_duration = 0.0
        if points:
            total_duration = points[-1].time_from_start
        
        # TrajectoryData 생성
        data = TrajectoryData(
            name=name,
            created_at=datetime.now().isoformat(),
            description=description,
            goal_position=goal_position,
            goal_orientation=goal_orientation,
            planner_id=planner_id,
            planning_group=planning_group,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
            joint_names=list(trajectory.joint_names),
            points=points,
            num_points=len(points),
            total_duration=round(total_duration, 6)
        )
        
        # YAML 데이터 준비
        yaml_data = self._to_yaml_dict(data)
        
        # 파일명 생성
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{name}_{timestamp}.yaml"
        
        if not filename.endswith('.yaml'):
            filename += '.yaml'
        
        filepath = os.path.join(self.output_dir, filename)
        
        # YAML 파일 저장
        with open(filepath, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        return filepath
    
    def save_batch(
        self,
        trajectories: List[Dict],
        filename: str = "batch_trajectories.yaml"
    ) -> str:
        """
        여러 trajectory를 하나의 YAML 파일로 저장
        
        Args:
            trajectories: trajectory 데이터 리스트
                각 항목은 {'trajectory': JointTrajectory, 'name': str, ...} 형태
            filename: 저장할 파일 이름
            
        Returns:
            저장된 파일 경로
        """
        batch_data = {
            'metadata': {
                'created_at': datetime.now().isoformat(),
                'num_trajectories': len(trajectories),
            },
            'trajectories': []
        }
        
        for i, traj_info in enumerate(trajectories):
            trajectory = traj_info.get('trajectory')
            name = traj_info.get('name', f'trajectory_{i}')
            
            points = []
            for point in trajectory.points:
                time_secs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                points.append({
                    'time_from_start': round(time_secs, 6),
                    'positions': [round(p, 6) for p in point.positions],
                    'velocities': [round(v, 6) for v in point.velocities] if point.velocities else [],
                    'accelerations': [round(a, 6) for a in point.accelerations] if point.accelerations else [],
                })
            
            traj_data = {
                'name': name,
                'goal_position': traj_info.get('goal_position'),
                'joint_names': list(trajectory.joint_names),
                'num_points': len(points),
                'total_duration': points[-1]['time_from_start'] if points else 0.0,
                'points': points
            }
            
            batch_data['trajectories'].append(traj_data)
        
        if not filename.endswith('.yaml'):
            filename += '.yaml'
        
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w') as f:
            yaml.dump(batch_data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        return filepath
    
    def _to_yaml_dict(self, data: TrajectoryData) -> Dict[str, Any]:
        """TrajectoryData를 YAML용 딕셔너리로 변환"""
        result = {
            'trajectory': {
                'metadata': {
                    'name': data.name,
                    'created_at': data.created_at,
                    'description': data.description,
                },
                'goal': {
                    'position': data.goal_position,
                    'orientation': data.goal_orientation,
                },
                'planning_config': {
                    'planner_id': data.planner_id,
                    'planning_group': data.planning_group,
                    'velocity_scaling': data.velocity_scaling,
                    'acceleration_scaling': data.acceleration_scaling,
                },
                'joint_info': {
                    'joint_names': data.joint_names,
                    'num_joints': len(data.joint_names) if data.joint_names else 0,
                },
                'statistics': {
                    'num_points': data.num_points,
                    'total_duration': data.total_duration,
                },
                'points': [asdict(p) for p in data.points] if data.points else []
            }
        }
        return result
    
    @staticmethod
    def load(filepath: str) -> Dict[str, Any]:
        """
        YAML 파일에서 trajectory 데이터 로드
        
        Args:
            filepath: YAML 파일 경로
            
        Returns:
            trajectory 데이터 딕셔너리
        """
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        return data
    
    @staticmethod
    def to_joint_trajectory_msg(data: Dict[str, Any]) -> JointTrajectory:
        """
        YAML 데이터를 JointTrajectory 메시지로 변환
        
        Args:
            data: YAML에서 로드한 데이터
            
        Returns:
            JointTrajectory 메시지
        """
        traj = JointTrajectory()
        
        traj_data = data.get('trajectory', data)
        
        # Joint names
        joint_info = traj_data.get('joint_info', {})
        traj.joint_names = joint_info.get('joint_names', [])
        
        # Points
        for point_data in traj_data.get('points', []):
            point = JointTrajectoryPoint()
            
            # Time from start
            time_secs = point_data.get('time_from_start', 0.0)
            point.time_from_start = Duration(
                sec=int(time_secs),
                nanosec=int((time_secs % 1) * 1e9)
            )
            
            point.positions = point_data.get('positions', [])
            point.velocities = point_data.get('velocities', [])
            point.accelerations = point_data.get('accelerations', [])
            point.effort = point_data.get('effort', [])
            
            traj.points.append(point)
        
        return traj
