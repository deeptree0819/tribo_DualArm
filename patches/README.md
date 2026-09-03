# MoveIt2 패치

이 디렉토리에는 SO-ARM101(5-DOF)에서 MoveIt2를 사용하기 위해 적용해야 하는 업스트림 `moveit2` 소스 패치가 들어 있습니다. 커스텀 패키지(`src/`)만으로는 적용되지 않으므로, MoveIt2를 소스로 빌드하는 경우 아래 패치를 직접 적용해야 합니다.

## `moveit2-position_only_ik-interactive-marker.patch`

**대상 파일:** `moveit_ros/planning/kinematics_plugin_loader/src/kinematics_plugin_loader.cpp`

**문제:** RViz의 인터랙티브 마커(그리퍼 구)를 드래그할 때, MoveIt이 5-DOF SO-ARM101에서 6-DOF full IK를 시도해 해를 찾지 못하고 마커가 따라오지 못함.

**원인:** RViz 노드가 파라미터 오버라이드로부터 솔버별 파라미터를 자동 declare하지 않아, `KinematicsBase::lookupParam()`이 `position_only_ik`를 읽지 못하고 무시됨.

**수정:** `KinematicsPluginLoader::getLoaderFunction()`에서 솔버별 `<group>.position_only_ik` 파라미터를 노드에 명시적으로 declare. 이로써 KDL 솔버가 position-only IK 모드로 동작하여 인터랙티브 마커 IK가 5-DOF에서도 풀린다.

## 적용 방법

MoveIt2(`jazzy` 브랜치)를 소스로 워크스페이스에 clone한 뒤:

```bash
cd ~/ros2_ws/src
git clone -b jazzy https://github.com/moveit/moveit2.git
cd moveit2
git apply /path/to/patches/moveit2-position_only_ik-interactive-marker.patch

# 적용 확인
git diff --stat
```

그 후 다시 빌드:

```bash
cd ~/ros2_ws
colcon build --packages-select moveit_ros_planning
```

> 브랜치에 따라 `kinematics_plugin_loader.cpp`의 라인 번호가 달라 `git apply`가 컨텍스트 불일치로 실패할 수 있습니다. 이 경우 3-way 병합 또는 `patch`로 적용하세요:
> ```bash
> git apply --3way /path/to/patches/moveit2-position_only_ik-interactive-marker.patch
> # 또는
> patch -p1 < /path/to/patches/moveit2-position_only_ik-interactive-marker.patch
> ```

> apt 바이너리(`ros-jazzy-moveit`)를 사용하는 경우 이 패치를 직접 적용할 수 없습니다. 인터랙티브 마커 IK가 필요하면 `moveit_ros_planning`을 소스로 빌드해야 합니다.
