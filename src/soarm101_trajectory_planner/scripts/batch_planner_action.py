#!/usr/bin/env python3
"""
batch_planner.py: YAML 설정 파일에서 여러 목표점을 읽어 순차적으로 경로를 계획합니다.

주의: 이 스크립트를 실행하기 전에 MoveIt을 먼저 실행해야 합니다:
    ros2 launch arm_moveit_config demo.launch.py

사용 예시:
    ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml
    ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml -o batch_result.yaml
"""

import argparse
import sys
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration

from soarm101_trajectory_planner.trajectory_saver import TrajectorySaver

import yaml
import numpy as np
from datetime import datetime
from typing import List, Dict, Any, Optional, Tuple


class BatchTrajectoryPlanner(Node):
    """여러 목표점에 대한 배치 경로 계획 (OMPL 서비스 기반)"""

    # SRDF에 정의된 named targets
    NAMED_TARGETS = {
        "home": {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.9,
            "elbow_flex": 1.2,
            "wrist_flex": 1.1,
            "wrist_roll": 0.0,
        },
        "zero": {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": 0.0,
        },
    }

    JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
    ]

    def __init__(
        self,
        planning_group: str = "arm",
        planner_id: str = "RRTConnect",
        velocity_scaling: float = 0.1,
        acceleration_scaling: float = 0.1,
        position_only: bool = True,
    ):
        super().__init__("batch_trajectory_planner")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.planning_group = planning_group
        self.planner_id = planner_id
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.position_only = position_only

        # Planning parameters
        self.planning_time = 5.0
        self.num_planning_attempts = 10
        self.base_frame = "base_link"
        self.end_effector_link = "gripper_link"

        # Current joint state
        self._current_joint_state = None

        # Callback group
        self._callback_group = ReentrantCallbackGroup()

        # Service client
        self._plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
            callback_group=self._callback_group,
        )

        # Joint state subscriber
        self._joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
            callback_group=self._callback_group,
        )

        self.get_logger().info("BatchTrajectoryPlanner 초기화 완료")

    def _joint_state_callback(self, msg: JointState):
        self._current_joint_state = msg

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
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
        self.get_logger().info("Joint state 대기 중...")
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._current_joint_state is not None:
                self.get_logger().info("Joint state 수신됨")
                return True

        self.get_logger().warn("Joint state를 받지 못했습니다. 기본값 사용.")
        self._current_joint_state = JointState()
        self._current_joint_state.name = self.JOINT_NAMES
        self._current_joint_state.position = [0.0] * len(self.JOINT_NAMES)
        return True

    # ------------------------------------------------------------------
    # Planning methods
    # ------------------------------------------------------------------

    def plan_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 1.57,
        yaw: float = 0.0,
        position_only: Optional[bool] = None,
        orientation_tolerance: float = 3.14,
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """Pose 목표로 경로 계획"""
        if position_only is None:
            position_only = self.position_only

        request = self._make_base_request()

        # Position constraint
        goal_constraints = Constraints()

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.weight = 1.0

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.02]  # 20mm tolerance

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose)

        position_constraint.constraint_region = bounding_volume
        goal_constraints.position_constraints.append(position_constraint)

        # Orientation constraint (optional)
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

        request.goal_constraints.append(goal_constraints)
        return self._call_plan_service(request)

    def plan_to_joint_target(
        self, joint_positions: Dict[str, float]
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """관절 목표로 경로 계획"""
        request = self._make_base_request()

        goal_constraints = Constraints()
        for joint_name, position in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        request.goal_constraints.append(goal_constraints)
        return self._call_plan_service(request)

    def plan_to_named_target(
        self, target_name: str
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """Named target으로 경로 계획"""
        if target_name not in self.NAMED_TARGETS:
            return (
                False,
                None,
                f"Unknown target: {target_name}. Available: {list(self.NAMED_TARGETS.keys())}",
            )
        return self.plan_to_joint_target(self.NAMED_TARGETS[target_name])

    # ------------------------------------------------------------------
    # Batch planning
    # ------------------------------------------------------------------

    def plan_waypoints(
        self, waypoints: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """
        여러 waypoint에 대해 순차적으로 경로 계획

        Args:
            waypoints: waypoint 리스트. 각 항목은 다음 형태:
                {'name': '...', 'position': {'x':..., 'y':..., 'z':...}, 'orientation': {...}}
                또는
                {'name': '...', 'target': 'home'}

        Returns:
            결과 리스트
        """
        results = []

        for i, waypoint in enumerate(waypoints):
            name = waypoint.get("name", f"waypoint_{i}")
            self.get_logger().info(f"\n[{i+1}/{len(waypoints)}] Planning: {name}")

            try:
                if "target" in waypoint:
                    success, trajectory, error = self.plan_to_named_target(
                        waypoint["target"]
                    )
                    goal_position = None
                else:
                    pos = waypoint.get("position", {})
                    ori = waypoint.get("orientation", {})

                    success, trajectory, error = self.plan_to_pose(
                        x=pos.get("x", 0.0),
                        y=pos.get("y", 0.0),
                        z=pos.get("z", 0.0),
                        roll=ori.get("roll", 0.0),
                        pitch=ori.get("pitch", 1.57),
                        yaw=ori.get("yaw", 0.0),
                    )
                    goal_position = pos

                result = {
                    "name": name,
                    "success": success,
                    "trajectory": trajectory,
                    "goal_position": goal_position,
                    "error": error,
                }

                if success:
                    self.get_logger().info(
                        f"  ✅ 성공 - Points: {len(trajectory.points)}"
                    )
                    # 성공 후 현재 joint state를 마지막 trajectory point로 갱신
                    self._update_joint_state_from_trajectory(trajectory)
                else:
                    self.get_logger().warn(f"  ❌ 실패 - {error}")

            except Exception as e:
                result = {
                    "name": name,
                    "success": False,
                    "trajectory": None,
                    "goal_position": None,
                    "error": str(e),
                }
                self.get_logger().error(f"  ❌ 오류 - {e}")

            results.append(result)

        return results

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _make_base_request(self) -> MotionPlanRequest:
        """공통 MotionPlanRequest 생성"""
        request = MotionPlanRequest()
        request.group_name = self.planning_group
        request.num_planning_attempts = self.num_planning_attempts
        request.allowed_planning_time = self.planning_time
        request.max_velocity_scaling_factor = self.velocity_scaling
        request.max_acceleration_scaling_factor = self.acceleration_scaling
        request.pipeline_id = "ompl"
        request.planner_id = self.planner_id

        if self._current_joint_state is not None:
            request.start_state.joint_state = self._current_joint_state
            request.start_state.is_diff = False

        return request

    def _call_plan_service(
        self, request: MotionPlanRequest
    ) -> Tuple[bool, Optional[JointTrajectory], str]:
        """Planning 서비스 호출"""
        self.get_logger().info(
            f"경로 계획 요청 중... (pipeline: {request.pipeline_id}, planner: {request.planner_id})"
        )

        srv_request = GetMotionPlan.Request()
        srv_request.motion_plan_request = request

        future = self._plan_client.call_async(srv_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is None:
            return False, None, "Planning service call failed (timeout or error)"

        response = future.result()

        if response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
            trajectory = response.motion_plan_response.trajectory.joint_trajectory
            self.get_logger().info(
                f"경로 계획 성공! Points: {len(trajectory.points)}"
            )
            return True, trajectory, ""
        else:
            error_code = response.motion_plan_response.error_code.val
            error_msg = f"Planning failed with error code: {error_code}"
            self.get_logger().error(error_msg)
            return False, None, error_msg

    def _update_joint_state_from_trajectory(self, trajectory: JointTrajectory):
        """성공한 trajectory의 마지막 point로 joint state 갱신 (다음 계획의 시작점)"""
        if trajectory.points:
            last_point = trajectory.points[-1]
            js = JointState()
            js.name = list(trajectory.joint_names)
            js.position = list(last_point.positions)
            self._current_joint_state = js

    @staticmethod
    def _rpy_to_quaternion(
        roll: float, pitch: float, yaw: float
    ) -> Tuple[float, float, float, float]:
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
    def wait_for_action_server(self, timeout_sec: float = 10.0) -> bool:
        self.get_logger().info("FollowJointTrajectory action 서버 대기 중...")
        if not self.client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error("FollowJointTrajectory action 서버에 연결할 수 없습니다.")
            self.get_logger().error("ros2 action list | grep follow_joint_trajectory 로 이름 확인하세요.")
            return False
        self.get_logger().info("Action 서버 연결됨")
        return True


    def send_trajectory_action(
        self,
        traj: JointTrajectory,
        goal_time_tolerance_sec: float = 0.5,
    ) -> bool:
        """MoveIt에서 나온 JointTrajectory를 FollowJointTrajectory 액션으로 실행"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        # (선택) controller가 tolerance를 보는 경우가 많아서 기본값 부여
        goal.goal_time_tolerance = Duration(sec=int(goal_time_tolerance_sec),
                                            nanosec=int((goal_time_tolerance_sec % 1.0) * 1e9))

        # goal 전송
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Action goal이 거절되었습니다.")
            return False

        self.get_logger().info("Action goal accepted. 실행 결과 대기 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=300.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error("Action result를 받지 못했습니다(timeout/error).")
            return False

        status = result.status  # 0~(rclpy action status)
        # FollowJointTrajectory.Result에는 error_code, error_string이 있음
        res = result.result
        self.get_logger().info(f"Action finished. status={status}, error_code={res.error_code}, msg='{res.error_string}'")

        return res.error_code == 0  # 0 == SUCCESS (대부분 컨트롤러에서)


def load_waypoints_config(config_path: str) -> Dict[str, Any]:
    """YAML 설정 파일 로드"""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def main():
    parser = argparse.ArgumentParser(
        description="SO-ARM101 Batch Trajectory Planner (OMPL)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
주의: 이 스크립트 실행 전에 MoveIt을 먼저 실행하세요:
    ros2 launch arm_moveit_config demo.launch.py

설정 파일 예시 (waypoints.yaml):
-------------------------------------------
planning:
  group: arm
  planner: RRTConnect
  velocity_scaling: 0.1
  acceleration_scaling: 0.1
  position_only: true

waypoints:
  - name: start_position
    target: home

  - name: pick_approach
    position:
      x: 0.15
      y: 0.10
      z: 0.15
    orientation:
      roll: 0.0
      pitch: 1.57
      yaw: 0.0

  - name: pick_position
    position:
      x: 0.15
      y: 0.10
      z: 0.05

  - name: return_home
    target: home
-------------------------------------------

사용 예시:
  ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml
  ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml -o result.yaml
        """,
    )

    parser.add_argument(
        "--config", "-c", type=str, required=True, help="Waypoints 설정 파일 경로 (YAML)"
    )
    parser.add_argument(
        "--output", "-o", type=str, default=None, help="출력 YAML 파일 이름"
    )
    parser.add_argument(
        "--output-dir",
        "-d",
        type=str,
        default="./trajectories",
        help="출력 디렉토리 (기본: ./trajectories)",
    )
    parser.add_argument(
        "--save-individual",
        "-s",
        action="store_true",
        help="각 trajectory를 개별 파일로도 저장",
    )

    args = parser.parse_args()

    # 설정 파일 로드
    if not os.path.exists(args.config):
        print(f"ERROR: 설정 파일을 찾을 수 없습니다: {args.config}")
        return 1

    config = load_waypoints_config(args.config)

    # Planning 설정 추출
    planning_config = config.get("planning", {})
    planning_group = planning_config.get("group", "arm")
    planner_id = planning_config.get("planner", "RRTConnect")
    velocity_scaling = planning_config.get("velocity_scaling", 0.1)
    acceleration_scaling = planning_config.get("acceleration_scaling", 0.1)
    position_only = planning_config.get("position_only", True)

    # Waypoints 추출
    waypoints = config.get("waypoints", [])
    if not waypoints:
        print("ERROR: waypoints가 정의되지 않았습니다.")
        return 1

    print(f"\n📋 Batch Planning 시작")
    print(f"   Waypoints: {len(waypoints)}개")
    print(f"   Planner: {planner_id} (OMPL)")
    print(f"   Group: {planning_group}")
    print(f"   Position only: {position_only}")

    # ROS2 초기화
    rclpy.init()

    try:
        # 노드 생성
        node = BatchTrajectoryPlanner(
            planning_group=planning_group,
            planner_id=planner_id,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
            position_only=position_only,
        )

        # 서비스 연결 대기
        if not node.wait_for_services(timeout_sec=10.0):
            print("\nERROR: MoveIt이 실행 중이지 않습니다.")
            print("먼저 다음 명령을 실행하세요:")
            print("  ros2 launch arm_moveit_config demo.launch.py")
            return 1

        # Joint state 대기
        node.wait_for_joint_state(timeout_sec=3.0)

        # 배치 경로 계획
        results = node.plan_waypoints(waypoints)

        # 결과 요약
        success_count = sum(1 for r in results if r["success"])
        print(f"\n📊 결과 요약: {success_count}/{len(results)} 성공")

        # 성공한 trajectory만 필터링
        successful_trajectories = [
            {
                "name": r["name"],
                "trajectory": r["trajectory"],
                "goal_position": r["goal_position"],
            }
            for r in results
            if r["success"]
        ]
        print("\n🚀 Action으로 trajectory 실행 시작...")
        for idx, traj_info in enumerate(successful_trajectories, start=1):
            name = traj_info["name"]
            traj = traj_info["trajectory"]

            # 안전: joint_names가 비어있으면 기본 JOINT_NAMES 강제(가끔 MoveIt 설정에 따라 비는 경우 대비)
            if not traj.joint_names:
                traj.joint_names = list(node.JOINT_NAMES)

            node.get_logger().info(f"[{idx}/{len(successful_trajectories)}] Execute: {name}")

            ok = node.send_trajectory_action(traj, goal_time_tolerance_sec=0.5)
            if not ok:
                node.get_logger().error(f"❌ 실행 실패: {name} (이후 trajectory 중단)")
                return 1


        if not successful_trajectories:
            print("WARNING: 성공한 경로 계획이 없습니다.")
            return 1

        # YAML로 저장
        saver = TrajectorySaver(output_dir=args.output_dir)

        # trajectory 포맷으로 배치 YAML 만들기
        from datetime import datetime
        import yaml

        created_at = datetime.now().isoformat()

        traj_entries = []
        for r in successful_trajectories:
            traj = r["trajectory"]
            joint_names = list(traj.joint_names)
            points = []
            for p in traj.points:
                t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                points.append({
                    "time_from_start": float(t),
                    "positions": list(p.positions),
                    "velocities": list(p.velocities),
                    "accelerations": list(p.accelerations),
                    "effort": list(p.effort),
                })

            total_duration = 0.0
            if traj.points:
                lp = traj.points[-1].time_from_start
                total_duration = lp.sec + lp.nanosec * 1e-9

            goal_pos = r.get("goal_position")  # batch에서는 orientation 정보가 없어서 position만 넣음

            traj_entries.append({
                "trajectory": {
                    "metadata": {
                        "name": r["name"],
                        "created_at": created_at,
                        "description": f"OMPL {planner_id} trajectory (batch)",
                    },
                    "goal": {
                        "position": goal_pos,  # 예: {'x':..., 'y':..., 'z':...} 또는 None
                        # orientation이 필요하면 여기 추가 (batch 입력에 없으면 기본값 넣기)
                    },
                    "planning_config": {
                        "planner_id": planner_id,
                        "planning_group": planning_group,
                        "velocity_scaling": velocity_scaling,
                        "acceleration_scaling": acceleration_scaling,
                    },
                    "joint_info": {
                        "joint_names": joint_names,
                        "num_joints": len(joint_names),
                    },
                    "statistics": {
                        "num_points": len(points),
                        "total_duration": float(total_duration),
                    },
                    "points": points,
                }
            })

        batch_like_trajectory_yaml = {
            "metadata": {
                "created_at": created_at,
                "num_trajectories": len(traj_entries),
            },
            "trajectories": traj_entries,
        }

        # 저장
        os.makedirs(args.output_dir, exist_ok=True)
        output_filename = args.output or f"trajectory_batch_{datetime.now().strftime('%Y%m%d_%H%M%S')}.yaml"
        out_path = os.path.join(args.output_dir, output_filename)

        with open(out_path, "w") as f:
            yaml.safe_dump(batch_like_trajectory_yaml, f, sort_keys=False, allow_unicode=True)

        print(f"\n✅ trajectory 포맷 배치 저장: {out_path}")


        # 개별 파일로 저장 (옵션)
        if args.save_individual:
            print("\n📁 개별 파일 저장 중...")
            for traj_info in successful_trajectories:
                filepath = saver.save(
                    trajectory=traj_info["trajectory"],
                    name=traj_info["name"],
                    goal_position=traj_info["goal_position"],
                    planner_id=planner_id,
                    planning_group=planning_group,
                )
                print(f"   - {filepath}")

        # Joint state 대기
        node.wait_for_joint_state(timeout_sec=3.0)
        # Action 서버 대기
        if not node.wait_for_action_server(timeout_sec=10.0):
            return 1


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


if __name__ == "__main__":
    sys.exit(main())
