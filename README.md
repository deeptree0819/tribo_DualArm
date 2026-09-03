# SO-ARM101 MoveIt2 Real Control

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![MoveIt2](https://img.shields.io/badge/MoveIt-2-orange)](https://moveit.picknik.ai/)
[![License](https://img.shields.io/badge/license-BSD--3--Clause-green)](#license)

MoveIt2와 PILZ Industrial Motion Planner를 사용해 [LeRobot **SO-ARM101**](https://github.com/huggingface/lerobot) 5-DOF 매니퓰레이터의 경로를 계획하고, 이를 **실제 로봇(Feetech 모터)** 에서 재생·제어하는 ROS 2 워크스페이스입니다.

> 워크플로우: **MoveIt2에서 Plan & Execute → 실제 SO-ARM101이 바로 구동** (YAML 기록/재생 단계 불필요)

> **현재 브랜치: `jazzy`** (Ubuntu 24.04 + ROS 2 Jazzy).

---

## 주요 기능

- **MoveIt 직접 구동** — RViz에서 Plan & Execute하면 실제 SO-ARM101이 바로 움직임. 시작 시 안전 자세 자동 정렬(`home_on_start`). (`soarm101_moveit_driver`)
- **MoveIt2 모션 플래닝** — PILZ Industrial Motion Planner (PTP / LIN) 기반 경로 계획
- **Trajectory 저장/재생** — 계획된 Joint Trajectory를 메타데이터와 함께 YAML로 직렬화
- **실제 로봇 제어** — LeRobot / Feetech 모터 버스를 통해 물리 SO-ARM101 구동
  - YAML 트래젝토리 재생 (가감속 S-curve 프로파일, 관절 부호/오프셋 보정)
  - IK 기반 실시간 제어 (`ikpy`)
  - 키보드 텔레오퍼레이션
- **RViz 시각화** — MoveIt 데모 및 인터랙티브 마커 기반 그리퍼 포즈 제어 (ver2 패치 필요)
- **그리퍼 포즈 모니터링** — 그리퍼(`gripper_link`)의 실시간 XYZ/RPY 좌표를 콘솔(`gripper_pose_monitor.py`) 또는 GUI(`gripper_pose_gui.py`)로 출력

---

## 패키지 구성

| 패키지 | 설명 |
|--------|------|
| `dt_arm_description` | SO-ARM101(40mm UP 버전) URDF / 메시 / 로봇 디스크립션 |
| `arm_moveit_config` | SO-ARM101용 MoveIt2 설정 (SRDF, kinematics, PILZ, 컨트롤러) |
| `dt_arm_moveit_config` | 대체 디스크립션(`dt_arm_description`) 기반 MoveIt2 설정 |
| `soarm101_trajectory_planner` | 경로 계획 + YAML 저장/재생 + 실제 로봇 제어 노드/스크립트 |
| `soarm101_moveit_driver` | **MoveIt 직접 구동 브리지** — RViz의 Plan & Execute가 실제 SO-ARM101을 바로 구동 (YAML 기록/재생 불필요). [README](src/soarm101_moveit_driver/README.md) |
| `patches/` | 업스트림 MoveIt2에 적용할 패치 (인터랙티브 마커 IK 수정) |

---

## 요구 사항

- Ubuntu 24.04 + [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- MoveIt 2 및 PILZ 플래너
- 실제 로봇 제어용: [LeRobot](https://github.com/huggingface/lerobot) (Feetech 모터 드라이버), `ikpy`, `pyserial`

```bash
# ROS 2 / MoveIt 의존성
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-pilz-industrial-motion-planner

# 실제 로봇 제어용 Python 의존성
pip install lerobot ikpy pyyaml numpy
```

> `ros-jazzy-moveit` 메타패키지가 빌드에 필요한 `moveit_ros_planning_interface`, `moveit_core` 등을 함께 설치합니다. 이걸 설치하지 않으면 `colcon build` 시
> `Could not find a package configuration file provided by "moveit_ros_planning_interface"` 에러가 납니다.

> ⚠️ 이 저장소에는 MoveIt2 소스 패키지가 포함되어 있지 않습니다. apt 바이너리(`ros-jazzy-moveit`)를 사용하거나, 소스 빌드가 필요하면 [moveit2](https://github.com/moveit/moveit2)를 워크스페이스에 추가로 clone 하세요. **인터랙티브 마커 IK를 쓰려면 `patches/`의 패치를 적용한 소스 빌드가 필요합니다.**

---

## 설치 및 빌드

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/deeptree0819/tribo_DualArm.git

# clone된 src 폴더 내용을 워크스페이스 src로 이동
cp -r soarm101_moveit2_real_control/src/* .

# 의존성 설치 (MoveIt / PILZ 등 package.xml에 선언된 의존성 자동 설치)
cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-select \
  dt_arm_description arm_moveit_config dt_arm_moveit_config \
  soarm101_trajectory_planner soarm101_moveit_driver
source install/setup.bash
```

> `rosdep`을 처음 쓰는 경우 `sudo rosdep init && rosdep update`를 한 번 실행하세요.
> rosdep 대신 직접 설치하려면 [요구 사항](#요구-사항)의 `sudo apt install ros-jazzy-moveit ros-jazzy-pilz-industrial-motion-planner`를 먼저 실행하면 됩니다.

### (선택) 인터랙티브 마커 IK용 MoveIt2 패치 적용

```bash
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/moveit/moveit2.git
cd moveit2
git apply ~/ros2_ws/src/soarm101_moveit2_real_control/patches/moveit2-position_only_ik-interactive-marker.patch
cd ~/ros2_ws && colcon build --packages-select moveit_ros_planning
```

자세한 내용은 [`patches/README.md`](patches/README.md) 참고.

---

## 사용법

> ### ⚡ 간단 실행 (권장) — MoveIt에서 바로 실제 로봇 구동
>
> YAML 기록/재생 없이, RViz에서 **Plan & Execute**만 누르면 실제 SO-ARM101이 움직입니다. (`soarm101_moveit_driver` 패키지)
>
> ```bash
> # 실제 로봇
> ros2 launch soarm101_moveit_driver real.launch.py port:=/dev/ttyACM0
> # 로봇 없이 RViz 파이프라인만 확인
> ros2 launch soarm101_moveit_driver real.launch.py dry_run:=true
> ```
>
> `lerobot`이 별도 venv에만 설치돼 있으면 `bridge_python:=<venv>/bin/python` 인자를 추가하세요.
> 연결 직후 자동으로 안전 자세(`home`)로 정렬되고, RViz에서 Plan & Execute하면 실제 팔이 움직입니다.
> 자세한 내용은 [`src/soarm101_moveit_driver/README.md`](src/soarm101_moveit_driver/README.md). 아래 1~3은 기존(YAML 기록/재생) 방식입니다.

### 1. MoveIt2 실행 (시뮬레이션 / 플래닝)

```bash
ros2 launch arm_moveit_config demo.launch.py
```

#### 1-1. 그리퍼 실시간 좌표 모니터링 (콘솔)

`demo.launch.py`가 실행 중인 상태에서 별도 터미널을 열어, 그리퍼(`gripper_link`)의 현재 XYZ/RPY를 콘솔에 실시간 출력합니다. RViz에서 Joints 슬라이더나 인터랙티브 마커로 팔을 움직이면 값이 즉시 갱신됩니다.

```bash
# 콘솔 출력 버전 (base_link -> gripper_link TF 기준)
ros2 run soarm101_trajectory_planner gripper_pose_monitor.py

# GUI 창 버전
ros2 run soarm101_trajectory_planner gripper_pose_gui.py
```

출력 예시:

```
Position (m): X=+0.1234  Y=-0.0456  Z=+0.2010  │ (mm): X=+123.4  Y=-45.6  Z=+201.0
Rotation (deg): R=+0.0°  P=+90.0°  Y=+0.0°
```

> 좌표는 `base_link`(로봇 베이스) 기준 `gripper_link`(엔드이펙터)의 위치/자세입니다. 0.5mm 미만 변화는 무시하여 같은 줄에서 갱신됩니다.

### 2. 경로 계획 후 YAML 저장

```bash
# XYZ 좌표 목표
ros2 run soarm101_trajectory_planner plan_trajectory.py --x 0.2 --y 0.1 --z 0.15

# Named target (SRDF 정의 포즈)
ros2 run soarm101_trajectory_planner plan_trajectory.py --target home

# 여러 waypoint 배치 계획
ros2 run soarm101_trajectory_planner batch_planner.py -c waypoints.yaml
```

자세한 옵션과 YAML 포맷은 [`src/soarm101_trajectory_planner/README.md`](src/soarm101_trajectory_planner/README.md) 참고.

### 3. 실제 SO-ARM101에서 재생

`play_so101/`에는 바로 실행해볼 수 있는 샘플 `trajectory.yaml`(소각도 안전 동작 → 0 복귀)이 포함되어 있습니다. 로봇은 먼저 모든 관절을 0°로 이동한 뒤 YAML 경로를 재생합니다.

```bash
cd ~/ros2_ws/src/soarm101_trajectory_planner/play_so101

# 빠른 시작: 동봉된 샘플 trajectory.yaml 재생 (포트는 환경에 맞게 변경)
python play_yaml_trajectory_so101.py --yaml trajectory.yaml --port /dev/so101_follower

# YAML 트래젝토리 재생 (포트는 환경에 맞게 변경)
python play_yaml_trajectory_so101.py --yaml trajectory.yaml --port /dev/ttyACM0

# 실제 모터 구동 없이 값만 확인 (dry-run)
python play_yaml_trajectory_so101.py --yaml trajectory.yaml --dry-run

# 관절 방향이 반대인 경우 부호 보정
python play_yaml_trajectory_so101.py --yaml trajectory.yaml \
    --joint-signs "shoulder_lift:-1,elbow_flex:-1"

# 캘리브레이션 ID 지정 (기본: my_follower_arm)
# ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json 를 자동 로드
python play_yaml_trajectory_so101.py --yaml trajectory.yaml --robot-id my_follower_arm

# 캘리브레이션 파일이 없을 때 새로 캘리브레이션 수행
python play_yaml_trajectory_so101.py --yaml trajectory.yaml --calibrate
```

> **참고 (LeRobot 0.5.x):** LeRobot 0.5.x부터 follower 모듈이 `lerobot.robots.so_follower`로
> 통합되었습니다(이전: `lerobot.robots.so101_follower`). 본 스크립트는 새 경로를 사용하며,
> 저장된 캘리브레이션 파일이 있으면 대화형 프롬프트 없이 자동 로드합니다.

### 4. 키보드 / IK 실시간 제어

```bash
python keyboard_so101_fixed.py      # 키보드 텔레오퍼레이션
python so101_ik_control.py          # IK 기반 실시간 제어
```

---

## 좌표계 및 Named Targets

```
        Z (위)
        |
        +------ Y (왼쪽)
       /
      X (앞)
```
Base link는 로봇 베이스 중심, end effector(`gripper_link`)의 위치가 목표 좌표입니다.

| Arm target | 설명 | Gripper target | 설명 |
|------------|------|----------------|------|
| `home` | 초기 대기 자세 | `open` | 그리퍼 열림 |
| `zero` | 모든 관절 0도 | `close` | 그리퍼 닫힘 |
| `setup` | 초기 셋업(들린) 자세 | | |

---

## 트러블슈팅

- **인터랙티브 마커가 안 따라옴 / IK 실패** — `patches/`의 MoveIt2 패치 적용 여부 확인 (5-DOF position-only IK)
- **No valid motion plan found** — 목표가 워크스페이스 내인지, 충돌 없는 경로가 가능한지 확인
- **IK solution not found** — 목표 orientation 도달 가능 여부, kinematics solver timeout 확인
- **MoveIt 연결 실패** — `ros2 node list | grep move_group`으로 move_group 실행 확인
- **시리얼 포트 권한** — `sudo usermod -aG dialout $USER` 후 재로그인, 포트(`/dev/ttyACM*`) 확인
- **`ModuleNotFoundError: No module named 'lerobot.robots.so101_follower'`** — LeRobot 0.5.x에서 모듈이 `lerobot.robots.so_follower`로 통합됨. 최신 스크립트로 업데이트하면 해결
- **연결 시 캘리브레이션 프롬프트에서 멈춤 / `EOFError`** — 저장된 캘리브레이션 파일이 없는 경우. `--calibrate`로 한 번 캘리브레이션하거나 `--robot-id`로 기존 파일 지정

---

## License

- `soarm101_trajectory_planner`, `arm_moveit_config`, `dt_arm_moveit_config`: BSD-3-Clause
- `dt_arm_description`: Apache-2.0
- `play_so101` 내 LeRobot 기반 스크립트: Apache-2.0 (© HuggingFace Inc.)

## Maintainer

deeptree (deeptree00@gmail.com)
