#!/usr/bin/env python3
"""
play_yaml_trajectory_so101_fixed.py

개선사항:
1. move_to_zero_position: 부드러운 가감속 프로파일 적용 (S-curve 방식)
2. 관절 방향(부호) 보정 옵션 추가 (--joint-signs)
3. 관절별 오프셋 보정 옵션 추가 (--joint-offsets)
4. 더 낮은 기본 kp 값과 속도 제한 추가

사용법:
  python play_yaml_trajectory_so101_fixed.py --yaml trajectory.yaml --port /dev/ttyACM0

  # 관절 방향이 반대인 경우 (예: shoulder_lift, elbow_flex가 반대)
  python play_yaml_trajectory_so101_fixed.py --yaml trajectory.yaml \\
      --joint-signs "shoulder_lift:-1,elbow_flex:-1"

  # 디버그 모드 (실제 모터 구동 없이 값만 출력)
  python play_yaml_trajectory_so101_fixed.py --yaml trajectory.yaml --dry-run
"""

import argparse
import time
import math
from typing import Dict, List, Tuple, Optional

import numpy as np

try:
    import yaml
except ImportError as e:
    raise RuntimeError("PyYAML이 필요합니다. `pip install pyyaml` 후 다시 실행하세요.") from e


# ================================================================================
# 관절 보정 설정 (필요시 수정)
# ================================================================================
# 각 관절의 부호: MoveIt2(URDF)와 실제 모터 방향이 다르면 -1로 설정
DEFAULT_JOINT_SIGNS = {
    "shoulder_pan": -1,
    "shoulder_lift": -1,
    "elbow_flex": 1,
    "wrist_flex": -1,
    "wrist_roll": 1,
    "gripper": 1,
}

# 각 관절의 오프셋 (degree): URDF의 0도와 실제 로봇의 0도가 다르면 설정
DEFAULT_JOINT_OFFSETS = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}


def parse_joint_config(config_str: str) -> Dict[str, float]:
    """
    "joint1:value1,joint2:value2" 형식의 문자열을 파싱
    """
    result = {}
    if not config_str:
        return result
    
    for item in config_str.split(","):
        item = item.strip()
        if ":" in item:
            joint, value = item.split(":", 1)
            result[joint.strip()] = float(value.strip())
    return result


def smooth_interpolate(start: float, end: float, progress: float) -> float:
    """
    S-curve (smoothstep) 보간: 시작과 끝에서 부드럽게 가감속
    progress: 0.0 ~ 1.0
    """
    # Smoothstep: 3t^2 - 2t^3
    t = max(0.0, min(1.0, progress))
    smooth_t = t * t * (3.0 - 2.0 * t)
    return start + (end - start) * smooth_t


def move_to_zero_position_smooth(
    robot,
    duration: float = 5.0,
    control_freq: int = 50,
    max_velocity_deg_per_sec: float = 30.0,
    dry_run: bool = False,
):
    """
    부드러운 S-curve 프로파일로 원점(0도)으로 이동
    
    Args:
        robot: SO101Follower 인스턴스
        duration: 최소 이동 시간 (초)
        control_freq: 제어 주파수 (Hz)
        max_velocity_deg_per_sec: 최대 각속도 (도/초)
        dry_run: True면 실제 모터 구동 없이 출력만
    """
    zero_positions = {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.0,
    }

    # 현재 위치 읽기
    current_obs = robot.get_observation()
    start_positions = {}
    for k, v in current_obs.items():
        if k.endswith(".pos"):
            start_positions[k.removesuffix(".pos")] = float(v)

    # 각 관절별 필요한 이동량 계산
    max_travel = 0.0
    for joint_name in zero_positions:
        if joint_name in start_positions:
            travel = abs(zero_positions[joint_name] - start_positions[joint_name])
            max_travel = max(max_travel, travel)

    # 최대 이동량에 따른 실제 소요 시간 계산
    min_time_for_velocity = max_travel / max_velocity_deg_per_sec if max_velocity_deg_per_sec > 0 else 0
    actual_duration = max(duration, min_time_for_velocity)

    total_steps = int(actual_duration * control_freq)
    step_time = 1.0 / control_freq

    print(f"[ZERO] 부드러운 이동 시작")
    print(f"       최대 이동량: {max_travel:.1f}°")
    print(f"       소요 시간: {actual_duration:.1f}s (최소 {duration}s, 속도제한 {min_time_for_velocity:.1f}s)")
    print(f"       제어 주파수: {control_freq}Hz")

    for step in range(total_steps + 1):
        progress = step / total_steps if total_steps > 0 else 1.0

        robot_action = {}
        for joint_name, target_deg in zero_positions.items():
            if joint_name in start_positions:
                start_deg = start_positions[joint_name]
                # S-curve 보간으로 부드럽게 이동
                cmd = smooth_interpolate(start_deg, target_deg, progress)
                robot_action[f"{joint_name}.pos"] = cmd

        if robot_action:
            if dry_run:
                if step % 25 == 0:
                    sample = ", ".join([f"{k.split('.')[0]}={v:.1f}°" for k, v in list(robot_action.items())[:3]])
                    print(f"  [DRY-RUN] progress={progress*100:.0f}% | {sample}")
            else:
                robot.send_action(robot_action)

        if step % (control_freq // 2) == 0 and not dry_run:
            print(f"  [ZERO] progress={progress*100:.0f}%")

        time.sleep(step_time)

    # 최종 위치 안정화 (P-control로 미세 조정)
    print("[ZERO] 최종 위치 안정화 중...")
    stabilize_steps = int(1.0 * control_freq)  # 1초간 안정화
    kp_stabilize = 0.3
    
    for _ in range(stabilize_steps):
        current_obs = robot.get_observation()
        current_positions = {}
        for k, v in current_obs.items():
            if k.endswith(".pos"):
                current_positions[k.removesuffix(".pos")] = float(v)

        robot_action = {}
        total_error = 0.0
        for joint_name, target_deg in zero_positions.items():
            if joint_name in current_positions:
                cur = current_positions[joint_name]
                err = target_deg - cur
                total_error += abs(err)
                cmd = cur + kp_stabilize * err
                cmd = max(-100.0, min(100.0, cmd))
                robot_action[f"{joint_name}.pos"] = cmd

        if not dry_run and robot_action:
            robot.send_action(robot_action)

        if total_error < 1.0:  # 모든 관절이 1도 이내면 완료
            break

        time.sleep(step_time)

    print("[ZERO] ✓ 완료")


def read_start_positions(robot) -> Dict[str, float]:
    """현재 로봇의 시작 관절각(도)을 읽어 저장"""
    obs = robot.get_observation()
    start_positions = {}
    for k, v in obs.items():
        if k.endswith(".pos"):
            start_positions[k.removesuffix(".pos")] = float(v)
    return start_positions


def return_to_start_position_smooth(
    robot,
    start_positions: Dict[str, float],
    duration: float = 4.0,
    control_freq: int = 50,
    max_velocity_deg_per_sec: float = 30.0,
    dry_run: bool = False,
):
    """부드러운 S-curve 프로파일로 시작 위치로 복귀"""
    
    current_obs = robot.get_observation()
    current_positions = {}
    for k, v in current_obs.items():
        if k.endswith(".pos"):
            current_positions[k.removesuffix(".pos")] = float(v)

    # 최대 이동량 계산
    max_travel = 0.0
    for joint_name in start_positions:
        if joint_name in current_positions:
            travel = abs(start_positions[joint_name] - current_positions[joint_name])
            max_travel = max(max_travel, travel)

    min_time_for_velocity = max_travel / max_velocity_deg_per_sec if max_velocity_deg_per_sec > 0 else 0
    actual_duration = max(duration, min_time_for_velocity)

    total_steps = int(actual_duration * control_freq)
    step_time = 1.0 / control_freq

    print(f"[RETURN] 시작 위치로 복귀 중... (소요 시간: {actual_duration:.1f}s)")

    for step in range(total_steps + 1):
        progress = step / total_steps if total_steps > 0 else 1.0

        robot_action = {}
        for joint_name, target_deg in start_positions.items():
            if joint_name in current_positions:
                start_deg = current_positions[joint_name]
                cmd = smooth_interpolate(start_deg, target_deg, progress)
                robot_action[f"{joint_name}.pos"] = cmd

        if robot_action and not dry_run:
            robot.send_action(robot_action)

        time.sleep(step_time)

    print("[RETURN] ✓ 완료")


# ================================================================================
# YAML trajectory 로딩 / 보간
# ================================================================================
def load_yaml_trajectories(yaml_path: str):
    """
    Returns:
      traj_list: List[Tuple[name:str, joint_names:List[str], times:(N,), positions:(N,J)]]
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    traj_dicts = []

    # 1) 단일 trajectory 파일: {trajectory: {...}}
    if isinstance(data, dict) and "trajectory" in data:
        traj_dicts = [data["trajectory"]]

    # 2) 배치 파일: {trajectories: [ {trajectory:{...}}, {...} ]}
    elif isinstance(data, dict) and "trajectories" in data:
        items = data["trajectories"] or []
        for item in items:
            if isinstance(item, dict) and "trajectory" in item:
                traj_dicts.append(item["trajectory"])
            else:
                # 혹시 item 자체가 trajectory 구조인 경우도 허용
                traj_dicts.append(item)

    # 3) 이미 trajectory dict 자체인 경우
    else:
        traj_dicts = [data]

    traj_list = []
    for i, traj in enumerate(traj_dicts):
        if not isinstance(traj, dict):
            raise ValueError(f"Trajectory #{i} 포맷이 dict가 아닙니다: {type(traj)}")

        if "joint_info" not in traj or "points" not in traj:
            raise ValueError(
                f"Trajectory #{i}에 joint_info/points가 없습니다. keys={list(traj.keys())}"
            )

        name = (
            (traj.get("metadata") or {}).get("name")
            or traj.get("name")
            or f"traj_{i}"
        )

        joint_names = traj["joint_info"]["joint_names"]
        points = traj["points"]

        times = np.array([float(p["time_from_start"]) for p in points], dtype=float)
        positions = np.array([p["positions"] for p in points], dtype=float)

        if times.ndim != 1 or positions.ndim != 2:
            raise ValueError(f"Trajectory #{i} points 포맷이 예상과 다릅니다.")
        if positions.shape[0] != times.shape[0]:
            raise ValueError(f"Trajectory #{i} times/positions 길이가 일치하지 않습니다.")
        if positions.shape[1] != len(joint_names):
            raise ValueError(f"Trajectory #{i} joint_names 개수와 positions 열 개수가 다릅니다.")

        order = np.argsort(times)
        times = times[order]
        positions = positions[order]

        traj_list.append((name, joint_names, times, positions))

    return traj_list



def interpolate_positions(times: np.ndarray, positions: np.ndarray, t: float) -> np.ndarray:
    """Linear interpolation."""
    if t <= times[0]:
        return positions[0]
    if t >= times[-1]:
        return positions[-1]

    idx = int(np.searchsorted(times, t, side="right") - 1)
    t0, t1 = times[idx], times[idx + 1]
    p0, p1 = positions[idx], positions[idx + 1]

    if t1 <= t0 + 1e-12:
        return p1

    alpha = (t - t0) / (t1 - t0)
    return (1.0 - alpha) * p0 + alpha * p1


def rad_to_deg(rad: float) -> float:
    return float(rad * 180.0 / math.pi)


def apply_joint_correction(
    joint_name: str,
    value_deg: float,
    joint_signs: Dict[str, float],
    joint_offsets: Dict[str, float],
) -> float:
    """
    관절 보정 적용: sign * value + offset
    """
    sign = joint_signs.get(joint_name, 1.0)
    offset = joint_offsets.get(joint_name, 0.0)
    return sign * value_deg + offset


def play_trajectory(
    robot,
    joint_names: List[str],
    times: np.ndarray,
    positions_rad: np.ndarray,
    control_freq: int = 50,
    kp: float = 0.3,
    keep_gripper_deg: float = 0.0,
    joint_signs: Optional[Dict[str, float]] = None,
    joint_offsets: Optional[Dict[str, float]] = None,
    dry_run: bool = False,
    verbose: bool = False,
):
    """
    trajectory 재생
    
    Args:
        joint_signs: 관절별 부호 보정 (MoveIt2 → 실제 모터)
        joint_offsets: 관절별 오프셋 보정 (degree)
        dry_run: True면 실제 모터 구동 없이 값만 출력
        verbose: 상세 로그 출력
    """
    if joint_signs is None:
        joint_signs = DEFAULT_JOINT_SIGNS.copy()
    if joint_offsets is None:
        joint_offsets = DEFAULT_JOINT_OFFSETS.copy()

    traj_joint_set = set(joint_names)

    dt = 1.0 / control_freq
    t_start = time.time()
    total_T = float(times[-1])
    
    print(f"[PLAY] 시작")
    print(f"       총 시간: {total_T:.3f}s")
    print(f"       제어 주파수: {control_freq}Hz, Kp: {kp}")
    print(f"       관절 부호: {joint_signs}")
    print(f"       관절 오프셋: {joint_offsets}")

    # 최종 목표 출력
    final_rad = positions_rad[-1]
    print(f"       최종 목표 (rad): {dict(zip(joint_names, final_rad))}")
    final_deg_corrected = {
        jn: apply_joint_correction(jn, rad_to_deg(final_rad[i]), joint_signs, joint_offsets)
        for i, jn in enumerate(joint_names)
    }
    print(f"       최종 목표 (deg, 보정후): {final_deg_corrected}")

    step = 0
    while True:
        now = time.time()
        t = now - t_start
        if t > total_T:
            break

        target_rad = interpolate_positions(times, positions_rad, t)
        
        # rad → deg 변환 후 관절 보정 적용
        target_deg_map = {}
        for i, jn in enumerate(joint_names):
            raw_deg = rad_to_deg(target_rad[i])
            corrected_deg = apply_joint_correction(jn, raw_deg, joint_signs, joint_offsets)
            target_deg_map[jn] = corrected_deg

        # 현재 상태 읽기
        if not dry_run:
            obs = robot.get_observation()
            current_deg = {}
            for k, v in obs.items():
                if k.endswith(".pos"):
                    current_deg[k.removesuffix(".pos")] = float(v)
        else:
            # dry-run 모드에서는 가상의 현재 위치 사용
            current_deg = {jn: target_deg_map[jn] for jn in joint_names}
            current_deg["gripper"] = keep_gripper_deg

        # P 제어로 command 생성
        robot_action: Dict[str, float] = {}

        for jn, tgt_deg in target_deg_map.items():
            if jn in current_deg:
                cur = current_deg[jn]
                err = tgt_deg - cur
                cmd = cur + kp * err
                # 안전 클램프
                cmd = max(-100.0, min(100.0, cmd))
                robot_action[f"{jn}.pos"] = cmd

        # gripper 유지
        if "gripper" in current_deg and "gripper" not in traj_joint_set:
            cur = current_deg["gripper"]
            err = keep_gripper_deg - cur
            cmd = cur + kp * err
            cmd = max(-100.0, min(100.0, cmd))
            robot_action["gripper.pos"] = cmd

        if robot_action:
            if dry_run:
                if step % (control_freq // 2) == 0:
                    sample = ", ".join([f"{k.split('.')[0]}={v:.1f}°" for k, v in list(robot_action.items())[:4]])
                    print(f"  [DRY-RUN] t={t:.2f}s | {sample}")
            else:
                robot.send_action(robot_action)

        if not dry_run and step % (control_freq // 5 if control_freq >= 5 else 1) == 0:
            sample = ", ".join([f"{k}={v:.1f}°" for k, v in list(target_deg_map.items())[:3]])
            print(f"  [PLAY] t={t:.2f}s | {sample}")

        step += 1
        time.sleep(dt)

    # 최종 포인트 안정화
    print("[PLAY] 최종 위치 안정화 중...")
    final_deg_map = {
        jn: apply_joint_correction(jn, rad_to_deg(positions_rad[-1, i]), joint_signs, joint_offsets)
        for i, jn in enumerate(joint_names)
    }
    
    stabilize_steps = int(1.5 * control_freq)
    for _ in range(stabilize_steps):
        if not dry_run:
            obs = robot.get_observation()
            current_deg = {k.removesuffix(".pos"): float(v) for k, v in obs.items() if k.endswith(".pos")}
        
            robot_action = {}
            total_error = 0.0
            for jn, tgt_deg in final_deg_map.items():
                if jn in current_deg:
                    cur = current_deg[jn]
                    err = tgt_deg - cur
                    total_error += abs(err)
                    cmd = cur + kp * err
                    cmd = max(-100.0, min(100.0, cmd))
                    robot_action[f"{jn}.pos"] = cmd
            
            if robot_action:
                robot.send_action(robot_action)
            
            if total_error < 2.0:
                break
        
        time.sleep(dt)

    print("[PLAY] ✓ 완료")


def print_joint_test_info():
    """관절 방향 테스트 방법 안내"""
    print("""
================================================================================
관절 방향 테스트 방법
================================================================================
RViz와 실제 로봇의 동작이 다른 경우, 각 관절의 방향(부호)이 맞는지 확인해야 합니다.

1. keyboard_so101_fixed.py로 로봇을 제어하면서 각 관절을 양의 방향(+)으로 움직여보세요.
2. RViz에서도 같은 관절을 양의 방향으로 움직여보세요.
3. 방향이 반대라면 해당 관절의 sign을 -1로 설정합니다.

예시:
  - shoulder_lift가 RViz에서는 위로, 실제 로봇에서는 아래로 움직인다면:
    --joint-signs "shoulder_lift:-1"
  
  - 여러 관절이 반대라면:
    --joint-signs "shoulder_lift:-1,elbow_flex:-1,wrist_flex:-1"

================================================================================
""")


def main():
    parser = argparse.ArgumentParser(
        description="SO-Arm101 YAML Trajectory Player (개선 버전)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
  # 기본 실행
  python %(prog)s --yaml trajectory.yaml
  
  # 관절 방향 보정 (shoulder_lift, elbow_flex가 반대인 경우)
  python %(prog)s --yaml trajectory.yaml --joint-signs "shoulder_lift:-1,elbow_flex:-1"
  
  # 디버그 모드 (실제 구동 없이 값만 출력)
  python %(prog)s --yaml trajectory.yaml --dry-run
  
  # 느린 속도로 실행
  python %(prog)s --yaml trajectory.yaml --max-vel 20 --kp 0.2
        """
    )
    parser.add_argument("--yaml", required=True, help="MoveIt2 trajectory YAML 파일 경로")
    parser.add_argument("--port", default="/dev/so101_follower", help="SO101 USB 포트")
    parser.add_argument("--calibrate", action="store_true", help="캘리브레이션 실행")
    parser.add_argument("--robot-id", default="my_follower_arm", help="로봇 ID (캘리브레이션 파일 이름)")
    parser.add_argument("--kp", type=float, default=0.3, help="P-control 게인 (기본: 0.3)")
    parser.add_argument("--freq", type=int, default=50, help="제어 주파수 Hz (기본: 50)")
    parser.add_argument("--keep-gripper", type=float, default=0.0, help="그리퍼 고정 각도 (도)")
    parser.add_argument("--max-vel", type=float, default=30.0, help="최대 각속도 (도/초, 기본: 30)")
    parser.add_argument("--zero-duration", type=float, default=5.0, help="원점 이동 최소 시간 (초)")
    
    # 관절 보정 옵션
    parser.add_argument(
        "--joint-signs",
        type=str,
        default="",
        help="관절 부호 보정 (예: 'shoulder_lift:-1,elbow_flex:-1')"
    )
    parser.add_argument(
        "--joint-offsets",
        type=str,
        default="",
        help="관절 오프셋 보정 - 도 단위 (예: 'shoulder_pan:5.0,wrist_flex:-3.0')"
    )
    
    # 디버그/테스트 옵션
    parser.add_argument("--dry-run", action="store_true", help="실제 구동 없이 값만 출력")
    parser.add_argument("--test-joints", action="store_true", help="관절 테스트 방법 안내 출력")
    parser.add_argument("--skip-zero", action="store_true", help="원점 이동 건너뛰기")
    parser.add_argument("--skip-return", action="store_true", help="시작 위치 복귀 건너뛰기")
    parser.add_argument("--verbose", action="store_true", help="상세 로그 출력")
    
    args = parser.parse_args()

    if args.test_joints:
        print_joint_test_info()
        return

    # 관절 보정 설정 파싱
    joint_signs = DEFAULT_JOINT_SIGNS.copy()
    joint_offsets = DEFAULT_JOINT_OFFSETS.copy()
    
    user_signs = parse_joint_config(args.joint_signs)
    user_offsets = parse_joint_config(args.joint_offsets)
    
    joint_signs.update(user_signs)
    joint_offsets.update(user_offsets)

    print("=" * 70)
    print("SO-Arm101 YAML Trajectory Player (개선 버전)")
    print("=" * 70)

    if args.dry_run:
        print("[MODE] DRY-RUN 모드 - 실제 모터 구동 없음")
        # dry-run 모드에서는 가짜 로봇 객체 사용
        class DummyRobot:
            def get_observation(self):
                return {
                    "shoulder_pan.pos": 0.0,
                    "shoulder_lift.pos": 0.0,
                    "elbow_flex.pos": 0.0,
                    "wrist_flex.pos": 0.0,
                    "wrist_roll.pos": 0.0,
                    "gripper.pos": 0.0,
                }
            def send_action(self, action):
                pass
            def connect(self):
                pass
            def calibrate(self):
                pass
            def disconnect(self):
                pass
        
        robot = DummyRobot()
        print("[CONNECT] DRY-RUN 모드 (가상 로봇)")
    else:
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

        print(f"[CONNECT] port={args.port}")
        robot_config = SO101FollowerConfig(port=args.port, id=args.robot_id)
        robot = SO101Follower(robot_config)
        # 대화형 캘리브레이션 프롬프트 없이 연결만 수행
        robot.connect(calibrate=False)
        if args.calibrate:
            # 새 캘리브레이션을 강제로 실행
            robot.calibrate()
        elif not robot.is_calibrated:
            # 저장된 캘리브레이션 파일(robot.calibration)을 모터에 자동 적용
            if robot.calibration:
                print(f"[CONNECT] 캘리브레이션 파일 적용: id={args.robot_id}")
                robot.bus.write_calibration(robot.calibration)
            else:
                print(f"[CONNECT] ⚠ 캘리브레이션 파일 없음(id={args.robot_id}). "
                      f"--calibrate 로 먼저 캘리브레이션하세요.")
        print("[CONNECT] ✓ 연결됨")

    try:
        if args.calibrate and not args.dry_run:
            print("[CALIB] 캘리브레이션 중...")
            robot.calibrate()
            print("[CALIB] ✓ 완료")
        else:
            print("[CALIB] 건너뜀 (기존 캘리브레이션 사용)")

        # 시작 자세 저장
        start_positions = read_start_positions(robot) if not args.dry_run else {}
        if start_positions:
            print("[START] 현재 위치:")
            for k in sorted(start_positions.keys()):
                print(f"  {k:15s}: {start_positions[k]:7.2f}°")

        # 원점 이동
        if not args.skip_zero:
            move_to_zero_position_smooth(
                robot,
                duration=args.zero_duration,
                control_freq=args.freq,
                max_velocity_deg_per_sec=args.max_vel,
                dry_run=args.dry_run,
            )
        else:
            print("[ZERO] 건너뜀")

        # YAML 로드 (단일/배치 모두 지원)
        traj_list = load_yaml_trajectories(args.yaml)
        print(f"[YAML] trajectories 개수: {len(traj_list)}")

        for idx, (traj_name, joint_names, times, positions_rad) in enumerate(traj_list, start=1):
            print(f"\n[PLAY] ({idx}/{len(traj_list)}) {traj_name}")
            print(f"[YAML] 관절: {joint_names}")
            print(f"[YAML] 포인트 수: {len(times)}, 총 시간: {times[-1]:.3f}s")

            play_trajectory(
                robot,
                joint_names=joint_names,
                times=times,
                positions_rad=positions_rad,
                control_freq=args.freq,
                kp=args.kp,
                keep_gripper_deg=args.keep_gripper,
                joint_signs=joint_signs,
                joint_offsets=joint_offsets,
                dry_run=args.dry_run,
                verbose=args.verbose,
            )


        # 시작 위치 복귀
        if not args.skip_return and start_positions:
            return_to_start_position_smooth(
                robot,
                start_positions,
                duration=4.0,
                control_freq=args.freq,
                max_velocity_deg_per_sec=args.max_vel,
                dry_run=args.dry_run,
            )

    finally:
        if not args.dry_run:
            robot.disconnect()
        print("[DISCONNECT] ✓ 완료")


if __name__ == "__main__":
    main()
