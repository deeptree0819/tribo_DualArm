# SO-ARM101 Trajectory Planner

MoveIt2 PILZ planner를 사용하여 LeRobot SO-ARM101의 경로를 계획하고 Joint Trajectory를 YAML 파일로 저장하는 ROS2 패키지입니다.

## 기능

- **PILZ Industrial Motion Planner 지원**
  - PTP (Point-to-Point): 가장 빠른 경로
  - LIN (Linear): 직선 경로
  - CIRC (Circular): 원형 경로 (지원 예정)

- **목표 지정 방식**
  - XYZ 좌표 + Roll/Pitch/Yaw 방향
  - SRDF에 정의된 Named Target (home, zero 등)
  - 관절 각도 직접 지정

- **YAML 출력**
  - Joint trajectory 전체 데이터 저장
  - 메타데이터 (목표, planner 설정, 통계)
  - 배치 모드 지원

## 설치

### 의존성

```bash
# ROS2 Jazzy 기준
sudo apt install ros-jazzy-moveit ros-jazzy-pilz-industrial-motion-planner
```

### 빌드

```bash
cd ~/ros2_ws/src
# 이 패키지와 arm_moveit_config 패키지가 있어야 함

cd ~/ros2_ws
colcon build --packages-select arm_moveit_config soarm101_trajectory_planner
source install/setup.bash
```

## 사용법

### 1. MoveIt 실행

먼저 별도 터미널에서 MoveIt을 실행합니다:

```bash
# 방법 1: arm_moveit_config의 demo launch 사용
ros2 launch arm_moveit_config demo.launch.py

# 방법 2: 이 패키지의 launch 사용
ros2 launch soarm101_trajectory_planner trajectory_planner.launch.py

# RViz 없이 실행
ros2 launch soarm101_trajectory_planner trajectory_planner.launch.py use_rviz:=false
```

### 2. 단일 목표점 경로 계획

```bash
# XYZ 좌표로 경로 계획
ros2 run soarm101_trajectory_planner plan_trajectory.py --x 0.2 --y 0.1 --z 0.15

# 방향까지 지정 (Roll, Pitch, Yaw - 라디안)
ros2 run soarm101_trajectory_planner plan_trajectory.py \
    --x 0.2 --y 0.0 --z 0.2 \
    --roll 0 --pitch 1.57 --yaw 0

# LIN planner 사용 (직선 경로)
ros2 run soarm101_trajectory_planner plan_trajectory.py \
    --x 0.2 --y 0.1 --z 0.15 \
    --planner LIN

# Named target 사용 (SRDF에 정의된 포즈)
ros2 run soarm101_trajectory_planner plan_trajectory.py --target home
ros2 run soarm101_trajectory_planner plan_trajectory.py --target zero

# 출력 파일명 지정
ros2 run soarm101_trajectory_planner plan_trajectory.py \
    --x 0.2 --y 0.1 --z 0.15 \
    -o my_trajectory.yaml

# 속도/가속도 스케일 조정
ros2 run soarm101_trajectory_planner plan_trajectory.py \
    --x 0.2 --y 0.1 --z 0.15 \
    --velocity-scale 0.3 --accel-scale 0.2
```

### 3. 배치 경로 계획 (여러 waypoint)

설정 파일 생성 (`waypoints.yaml`):

```yaml
planning:
  group: arm
  planner: PTP
  velocity_scaling: 0.1
  acceleration_scaling: 0.1

waypoints:
  - name: start_position
    target: home
    
  - name: pick_approach
    position:
      x: 0.20
      y: 0.10
      z: 0.15
    orientation:
      roll: 0.0
      pitch: 1.57
      yaw: 0.0
      
  - name: pick_position
    position:
      x: 0.20
      y: 0.10
      z: 0.05
      
  - name: return_home
    target: home
```

실행:

```bash
# 배치 경로 계획
ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml

# 출력 파일 지정
ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml -o batch_result.yaml

# 개별 파일로도 저장
ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml --save-individual
```

## 출력 YAML 형식

### 단일 Trajectory

```yaml
trajectory:
  metadata:
    name: pick_motion
    created_at: '2024-01-15T10:30:00'
    description: PILZ PTP trajectory
  goal:
    position:
      x: 0.2
      y: 0.1
      z: 0.15
    orientation:
      roll: 0.0
      pitch: 1.57
      yaw: 0.0
  planning_config:
    planner_id: PTP
    planning_group: arm
    velocity_scaling: 0.1
    acceleration_scaling: 0.1
  joint_info:
    joint_names:
      - shoulder_pan
      - shoulder_lift
      - elbow_flex
      - wrist_flex
      - wrist_roll
    num_joints: 5
  statistics:
    num_points: 50
    total_duration: 2.5
  points:
    - time_from_start: 0.0
      positions: [0.0, 0.0, 0.0, 0.0, 0.0]
      velocities: [0.0, 0.0, 0.0, 0.0, 0.0]
      accelerations: [0.0, 0.0, 0.0, 0.0, 0.0]
      effort: []
    - time_from_start: 0.05
      positions: [0.01, 0.02, 0.015, 0.01, 0.0]
      velocities: [0.1, 0.2, 0.15, 0.1, 0.0]
      accelerations: [1.0, 2.0, 1.5, 1.0, 0.0]
      effort: []
    # ... 더 많은 points
```

### 배치 Trajectory

```yaml
metadata:
  created_at: '2024-01-15T10:30:00'
  num_trajectories: 4
trajectories:
  - name: start_position
    goal_position: null  # named target 사용
    joint_names: [shoulder_pan, shoulder_lift, ...]
    num_points: 30
    total_duration: 1.5
    points:
      - time_from_start: 0.0
        positions: [...]
        velocities: [...]
      # ...
  - name: pick_approach
    goal_position:
      x: 0.2
      y: 0.1
      z: 0.15
    # ...
```

## Python API 사용

```python
import rclpy
from soarm101_trajectory_planner import TrajectoryPlanner, TrajectorySaver

# ROS2 초기화
rclpy.init()

# Planner 생성 및 초기화
planner = TrajectoryPlanner(
    planning_group="arm",
    planner_id="PTP",
    velocity_scaling=0.1
)
planner.initialize()

# 경로 계획
result = planner.plan_to_position(x=0.2, y=0.1, z=0.15, pitch=1.57)

if result.success:
    # YAML로 저장
    saver = TrajectorySaver(output_dir="./trajectories")
    filepath = saver.save(
        trajectory=result.joint_trajectory,
        name="pick_motion",
        goal_position={'x': 0.2, 'y': 0.1, 'z': 0.15}
    )
    print(f"저장됨: {filepath}")
else:
    print(f"실패: {result.error_message}")

# YAML 로드
data = TrajectorySaver.load(filepath)
trajectory_msg = TrajectorySaver.to_joint_trajectory_msg(data)

rclpy.shutdown()
```

## SO-ARM101 좌표계

```
        Z (위)
        |
        |
        +------ Y (왼쪽)
       /
      /
     X (앞)

Base link는 로봇 베이스의 중심에 위치합니다.
End effector (gripper_link)의 위치가 목표 좌표가 됩니다.
```

## SRDF Named Targets

`arm_moveit_config`에 정의된 named targets:

| 이름 | 설명 |
|------|------|
| `home` | 초기 대기 자세 |
| `zero` | 모든 관절 0도 |

Gripper:
| 이름 | 설명 |
|------|------|
| `open` | 그리퍼 열림 |
| `close` | 그리퍼 닫힘 |

## 트러블슈팅

### "No valid motion plan found"

- 목표 위치가 로봇의 workspace 내에 있는지 확인
- 충돌 없는 경로가 가능한지 확인
- 시작 상태가 유효한지 확인

### "IK solution not found"

- 목표 방향(orientation)이 도달 가능한지 확인
- end effector 링크가 올바른지 확인
- kinematics solver timeout 증가 시도

### MoveIt 연결 실패

1. MoveIt move_group이 실행 중인지 확인:
   ```bash
   ros2 node list | grep move_group
   ```

2. Planning group 이름 확인:
   ```bash
   ros2 param get /move_group robot_description_semantic
   ```

## 라이선스

BSD-3-Clause

## 작성자

deeptree (deeptree00@gmail.com)
