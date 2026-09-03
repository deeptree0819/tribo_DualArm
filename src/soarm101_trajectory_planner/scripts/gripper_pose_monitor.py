#!/usr/bin/env python3
"""
gripper_pose_monitor.py: 그리퍼의 실시간 XYZ 좌표를 모니터링합니다.

MoveIt RViz에서 Joints 슬라이더를 움직일 때 좌표가 실시간으로 업데이트됩니다.

사용법:
    ros2 run soarm101_trajectory_planner gripper_pose_monitor.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import DisplayRobotState, RobotState
from sensor_msgs.msg import JointState

import math
import sys
import os

# ANSI 색상 코드
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


def clear_line():
    """현재 줄 지우기"""
    sys.stdout.write('\r\033[K')
    sys.stdout.flush()


def quaternion_to_rpy(x, y, z, w):
    """Quaternion을 Roll, Pitch, Yaw로 변환 (라디안)"""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class GripperPoseMonitor(Node):
    def __init__(self):
        super().__init__('gripper_pose_monitor')
        
        # TF2 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 프레임 이름
        self.base_frame = 'base_link'
        self.gripper_frame = 'gripper_link'
        
        # 마지막 좌표 저장 (변화 감지용)
        self.last_pos = None
        self.last_joints = None
        
        # Display Robot State 구독 (RViz에서 슬라이더 조작 시)
        qos = QoSProfile(depth=10)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        self.display_state_sub = self.create_subscription(
            DisplayRobotState,
            '/display_robot_state',  # MoveIt RViz가 publish하는 토픽
            self.display_state_callback,
            10
        )
        
        # Joint States 구독 (실제 로봇 또는 시뮬레이션)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 타이머로 TF 주기적 확인
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # 초기 헤더 출력
        self.print_header()
        
    def print_header(self):
        """헤더 출력"""
        print(f"\n{Colors.BOLD}{'='*70}{Colors.ENDC}")
        print(f"{Colors.HEADER}  SO-ARM101 Gripper Pose Monitor{Colors.ENDC}")
        print(f"{Colors.BOLD}{'='*70}{Colors.ENDC}")
        print(f"  Base Frame: {self.base_frame}")
        print(f"  Gripper Frame: {self.gripper_frame}")
        print(f"{'='*70}\n")
        
    def display_state_callback(self, msg: DisplayRobotState):
        """RViz에서 로봇 상태가 변경될 때 호출"""
        # DisplayRobotState에서 joint positions 추출
        if msg.state.joint_state.name:
            self.process_joint_state(msg.state.joint_state)
    
    def joint_state_callback(self, msg: JointState):
        """Joint state 콜백"""
        self.process_joint_state(msg)
    
    def process_joint_state(self, joint_state: JointState):
        """Joint state 처리 및 출력"""
        # Joint positions를 딕셔너리로 변환
        joints = {}
        for i, name in enumerate(joint_state.name):
            if i < len(joint_state.position):
                joints[name] = joint_state.position[i]
        
        # 변화 감지 (너무 자주 출력 방지)
        if self.last_joints is not None:
            changed = False
            for name, pos in joints.items():
                if name in self.last_joints:
                    if abs(pos - self.last_joints[name]) > 0.001:  # 0.001 rad 이상 변화
                        changed = True
                        break
            if not changed:
                return
        
        self.last_joints = joints.copy()
    
    def timer_callback(self):
        """주기적으로 TF에서 그리퍼 위치 확인"""
        try:
            # base_link → gripper_link 변환 가져오기
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.gripper_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 위치 추출
            pos = transform.transform.translation
            rot = transform.transform.rotation
            
            # 변화 감지
            current_pos = (pos.x, pos.y, pos.z)
            if self.last_pos is not None:
                dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(current_pos, self.last_pos)))
                if dist < 0.0005:  # 0.5mm 미만 변화는 무시
                    return
            
            self.last_pos = current_pos
            
            # RPY 계산
            roll, pitch, yaw = quaternion_to_rpy(rot.x, rot.y, rot.z, rot.w)
            
            # 출력
            self.print_pose(pos.x, pos.y, pos.z, roll, pitch, yaw)
            
        except Exception as e:
            pass  # TF를 아직 받지 못한 경우 무시
    
    def print_pose(self, x, y, z, roll, pitch, yaw):
        """좌표 출력 (같은 줄에 업데이트)"""
        # 미터 → 밀리미터 변환 옵션
        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000
        
        # 라디안 → 도 변환
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        # 출력 형식
        output = (
            f"{Colors.CYAN}Position (m):{Colors.ENDC} "
            f"{Colors.GREEN}X={x:+.4f}{Colors.ENDC}  "
            f"{Colors.RED}Y={y:+.4f}{Colors.ENDC}  "
            f"{Colors.BLUE}Z={z:+.4f}{Colors.ENDC}  "
            f"{Colors.YELLOW}│{Colors.ENDC} "
            f"{Colors.CYAN}(mm):{Colors.ENDC} "
            f"X={x_mm:+.1f}  Y={y_mm:+.1f}  Z={z_mm:+.1f}"
        )
        
        output2 = (
            f"{Colors.CYAN}Rotation (deg):{Colors.ENDC} "
            f"R={roll_deg:+.1f}°  P={pitch_deg:+.1f}°  Y={yaw_deg:+.1f}°"
        )
        
        # 줄 지우고 새로 출력
        clear_line()
        print(output)
        clear_line()
        print(output2)
        
        # 커서를 위로 이동 (다음 업데이트가 같은 위치에 출력되도록)
        sys.stdout.write('\033[2A')
        sys.stdout.flush()


def main():
    rclpy.init()
    
    print(f"\n{Colors.YELLOW}Starting Gripper Pose Monitor...{Colors.ENDC}")
    print(f"{Colors.YELLOW}Move the joints in RViz to see real-time coordinates.{Colors.ENDC}")
    print(f"{Colors.YELLOW}Press Ctrl+C to exit.{Colors.ENDC}\n")
    
    node = GripperPoseMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n\n{Colors.YELLOW}Monitor stopped.{Colors.ENDC}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
