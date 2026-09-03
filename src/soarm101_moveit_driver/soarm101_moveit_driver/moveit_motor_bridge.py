#!/usr/bin/env python3
"""
moveit_motor_bridge.py — MoveIt2 <-> 실제 SO-ARM101 (Feetech/LeRobot) 직접 구동 브리지.

RViz의 "Plan & Execute"가 보내는 trajectory를 실제 모터로 바로 흘려보내고,
실제 모터 엔코더를 /joint_states로 publish 한다. 따라서 YAML로 따로 기록/재생할
필요 없이, MoveIt에서 계획하고 실행하면 실제 팔이 즉시 움직인다.

제공하는 인터페이스 (arm_moveit_config/config/moveit_controllers.yaml 과 매칭):
  - action  /arm_trajectory_controller/follow_joint_trajectory  (FollowJointTrajectory)
  - action  /gripper_action_controller/gripper_cmd               (GripperCommand)
  - topic   /joint_states                                        (sensor_msgs/JointState)

단위 변환: MoveIt/URDF는 radian, LeRobot 모터는 degree.
  motor_deg = sign * deg(rad) + offset_deg
  rad       = radian((motor_deg - offset_deg) / sign)

사용법:
  ros2 run soarm101_moveit_driver moveit_motor_bridge --ros-args -p port:=/dev/ttyACM0
  # 로봇 없이 파이프라인만 테스트 (가상 모터):
  ros2 run soarm101_moveit_driver moveit_motor_bridge --ros-args -p dry_run:=true
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from builtin_interfaces.msg import Duration  # noqa: F401  (문서용)
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER_JOINT = "gripper"
ALL_JOINTS = ARM_JOINTS + [GRIPPER_JOINT]

# MoveIt(URDF) 방향/0점과 실제 모터가 다를 때 보정 (play_yaml 기본값과 동일)
DEFAULT_SIGNS = {
    "shoulder_pan": -1.0,
    "shoulder_lift": -1.0,
    "elbow_flex": 1.0,
    "wrist_flex": -1.0,
    "wrist_roll": 1.0,
    "gripper": 1.0,
}
DEFAULT_OFFSETS_DEG = {j: 0.0 for j in ALL_JOINTS}

# 시작 시 이동할 안전한 범위-내 자세(rad). 모델의 관절 소프트리밋 안쪽으로 잡는다.
# (shoulder_lift 한계 ±1.745, elbow_flex 한계 ±1.69)
DEFAULT_HOME = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 1.5,
    "elbow_flex": 1.5,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
}


class MoveItMotorBridge(Node):
    def __init__(self):
        super().__init__("moveit_motor_bridge")

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.robot_id = self.declare_parameter("robot_id", "my_follower_arm").value
        self.dry_run = self.declare_parameter("dry_run", False).value
        self.control_freq = float(self.declare_parameter("control_freq", 50.0).value)
        self.state_pub_rate = float(self.declare_parameter("state_pub_rate", 30.0).value)
        self.max_step_deg = float(self.declare_parameter("max_step_deg", 6.0).value)

        self.signs = {
            j: float(self.declare_parameter(f"signs.{j}", DEFAULT_SIGNS[j]).value)
            for j in ALL_JOINTS
        }
        self.offsets_deg = {
            j: float(self.declare_parameter(f"offsets_deg.{j}", DEFAULT_OFFSETS_DEG[j]).value)
            for j in ALL_JOINTS
        }

        # 시작 시 안전 자세로 자동 이동 (시작 자세가 관절 한계를 벗어나 플래닝이
        # 막히는 것을 방지)
        self.home_on_start = bool(self.declare_parameter("home_on_start", True).value)
        self.home_duration = float(self.declare_parameter("home_duration", 4.0).value)
        self.home = {
            j: float(self.declare_parameter(f"home.{j}", DEFAULT_HOME[j]).value)
            for j in ARM_JOINTS
        }

        # ── 상태 ────────────────────────────────────────────────────────────
        self._lock = threading.Lock()           # 시리얼 버스 보호
        self._state_rad = {j: 0.0 for j in ALL_JOINTS}  # 최근 위치 캐시(표시용)
        self._executing = False                  # trajectory 실행 중에는 엔코더 read 중단
        self.robot = None

        self._connect_robot()

        # ── 인터페이스 ──────────────────────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / self.state_pub_rate, self._publish_joint_states, callback_group=cb)

        self._arm_server = ActionServer(
            self, FollowJointTrajectory,
            "/arm_trajectory_controller/follow_joint_trajectory",
            execute_callback=self._execute_arm,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self._gripper_server = ActionServer(
            self, GripperCommand,
            "/gripper_action_controller/gripper_cmd",
            execute_callback=self._execute_gripper,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        if self.home_on_start:
            self._go_home()

        mode = "DRY-RUN (가상 모터)" if self.dry_run else f"port={self.port}"
        self.get_logger().info(
            f"MoveIt motor bridge 준비 완료 [{mode}]. RViz에서 Plan & Execute 하세요.")

    # ────────────────────────────────────────────────────────────────────────
    # 로봇 연결
    # ────────────────────────────────────────────────────────────────────────
    def _connect_robot(self):
        if self.dry_run:
            self.get_logger().warn("dry_run=true: 실제 모터에 연결하지 않습니다 (RViz 표시만).")
            return
        try:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        except ImportError as e:
            self.get_logger().error(
                f"lerobot import 실패 ({e}). `pip install lerobot` 또는 dry_run:=true 로 실행하세요.")
            raise
        cfg = SO101FollowerConfig(port=self.port, id=self.robot_id)
        self.robot = SO101Follower(cfg)
        self.robot.connect(calibrate=False)
        self.get_logger().info(f"SO-ARM101 연결됨 (port={self.port}, id={self.robot_id}).")
        # 시작 위치를 캐시에 반영
        self._read_into_cache()

    # ────────────────────────────────────────────────────────────────────────
    # 시작 시 안전 자세로 이동
    # ────────────────────────────────────────────────────────────────────────
    def _go_home(self):
        """현재 위치에서 home 자세로 smoothstep 보간하며 부드럽게 이동."""
        if self.robot is not None:
            try:
                self._read_into_cache()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"home 전 위치 read 실패: {e}")
        start = {j: self._state_rad[j] for j in ARM_JOINTS}
        self.get_logger().info(
            f"home_on_start: {self.home_duration:.1f}s 동안 안전 자세로 이동합니다. "
            "(주변 공간 확인)")
        self._executing = True
        try:
            dt = 1.0 / self.control_freq
            t = 0.0
            while t < self.home_duration:
                a = t / self.home_duration
                sa = a * a * (3.0 - 2.0 * a)  # smoothstep
                self._send_rad({j: start[j] + sa * (self.home[j] - start[j]) for j in ARM_JOINTS})
                t += dt
                time.sleep(dt)
            self._send_rad(dict(self.home))
        finally:
            self._executing = False
        self.get_logger().info("home 자세 도달.")

    # ────────────────────────────────────────────────────────────────────────
    # 단위 변환
    # ────────────────────────────────────────────────────────────────────────
    def _rad_to_motor_deg(self, joint, rad):
        return self.signs[joint] * math.degrees(rad) + self.offsets_deg[joint]

    def _motor_deg_to_rad(self, joint, deg):
        return math.radians((deg - self.offsets_deg[joint]) / self.signs[joint])

    # ────────────────────────────────────────────────────────────────────────
    # 모터 I/O (락으로 보호)
    # ────────────────────────────────────────────────────────────────────────
    def _send_rad(self, positions_rad: dict):
        """positions_rad: {joint: rad} 를 모터로 전송하고 표시 캐시를 갱신."""
        action = {f"{j}.pos": self._rad_to_motor_deg(j, r) for j, r in positions_rad.items()}
        with self._lock:
            if self.robot is not None:
                self.robot.send_action(action)
        self._state_rad.update(positions_rad)

    def _read_into_cache(self):
        if self.robot is None:
            return
        with self._lock:
            obs = self.robot.get_observation()
        for k, v in obs.items():
            if k.endswith(".pos"):
                j = k[:-4]
                if j in self.signs:
                    self._state_rad[j] = self._motor_deg_to_rad(j, float(v))

    def _publish_joint_states(self):
        # 실행 중이 아니면 실제 엔코더를 읽어 캐시 갱신(시리얼 경합 방지)
        if self.robot is not None and not self._executing:
            try:
                self._read_into_cache()
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"joint_states read 실패: {e}", throttle_duration_sec=5.0)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ALL_JOINTS)
        msg.position = [self._state_rad[j] for j in ALL_JOINTS]
        self.joint_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────────
    # FollowJointTrajectory (arm)
    # ────────────────────────────────────────────────────────────────────────
    def _execute_arm(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        pts = list(traj.points)
        result = FollowJointTrajectory.Result()

        if not pts:
            self.get_logger().warn("빈 trajectory 수신.")
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result

        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts]
        total = times[-1] if times[-1] > 0 else 0.0
        dt = 1.0 / self.control_freq

        self.get_logger().info(
            f"trajectory 실행: {len(pts)} points, {total:.2f}s, joints={names}")
        self._executing = True
        try:
            t = 0.0
            while t < total:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    self.get_logger().warn("trajectory 취소됨.")
                    return result
                self._send_rad(self._sample(names, pts, times, t))
                t += dt
                time.sleep(dt)
            # 마지막 포인트 정확히
            self._send_rad({n: pts[-1].positions[i] for i, n in enumerate(names)})
        finally:
            self._executing = False

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("trajectory 실행 완료.")
        return result

    @staticmethod
    def _sample(names, pts, times, t):
        """시각 t(초)에서의 관절 위치(rad)를 선형 보간으로 구한다."""
        # 구간 찾기
        hi = 0
        while hi < len(times) and times[hi] < t:
            hi += 1
        if hi == 0:
            p = pts[0]
            return {n: p.positions[i] for i, n in enumerate(names)}
        if hi >= len(pts):
            p = pts[-1]
            return {n: p.positions[i] for i, n in enumerate(names)}
        t0, t1 = times[hi - 1], times[hi]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        p0, p1 = pts[hi - 1], pts[hi]
        return {
            n: p0.positions[i] + a * (p1.positions[i] - p0.positions[i])
            for i, n in enumerate(names)
        }

    # ────────────────────────────────────────────────────────────────────────
    # GripperCommand
    # ────────────────────────────────────────────────────────────────────────
    def _execute_gripper(self, goal_handle):
        target_rad = goal_handle.request.command.position  # 그리퍼 관절 목표(rad)
        self.get_logger().info(f"gripper 목표: {target_rad:.3f} rad")
        self._executing = True
        try:
            self._send_rad({GRIPPER_JOINT: float(target_rad)})
        finally:
            self._executing = False
        goal_handle.succeed()
        result = GripperCommand.Result()
        result.position = float(target_rad)
        result.reached_goal = True
        result.stalled = False
        return result

    # ────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            if self.robot is not None:
                self.robot.disconnect()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main():
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init()
    node = MoveItMotorBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()          # robot.disconnect() 포함
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
