#!/usr/bin/env python3
"""
Standalone Arm Controller - ROS2 의존성 제거된 순수 Python 버전
IKPy 기반 역기구학 및 궤적 보간 제어
"""

import ikpy.chain
import numpy as np
import time
from typing import Tuple, Optional


class ArmControllerNode:
    def __init__(self, urdf_path: str):
        """
        팔 제어기 초기화
        
        Args:
            urdf_path: URDF 파일 경로
        """
        # URDF 로드 및 IKPy 체인 구성
        # 4DOF IK: Pan, Lift, Elbow, Flex만 IK로 계산 (Roll, Gripper 제외)
        active_mask = [False, True, True, True, True, False, False]
        
        try:
            self.chain = ikpy.chain.Chain.from_urdf_file(
                urdf_path,
                base_elements=["base_link"],
                active_links_mask=active_mask
            )
            print(f"✅ URDF loaded successfully: {urdf_path}")
        except FileNotFoundError:
            raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        except Exception as e:
            raise RuntimeError(f"Failed to load URDF: {e}")

        # URDF에서 각 관절의 한계값 미리 추출
        self.joint_bounds = {}
        for i, link in enumerate(self.chain.links):
            if link.joint_type == 'revolute':
                self.joint_bounds[i] = link.bounds
            else:
                self.joint_bounds[i] = None
        
        self._print_joint_bounds()

        # --- 로봇의 현재 상태 저장소 ---
        self.current_ik_joints = np.zeros(len(self.chain.links)) 
        
        # 보간(Trajectory) 이동을 위한 변수들
        self.start_ik_joints = np.zeros(len(self.chain.links))
        self.target_ik_joints = np.zeros(len(self.chain.links))
        self.is_moving = False
        self.move_start_time = 0.0
        self.move_duration = 2.0

        # 개별 제어 관절값 초기화
        self.manual_flex_override = None
        self.current_wrist_roll = 0.0
        self.current_gripper = 0.0
        self.speed = 1.0
        self.target_dir = np.array([0.0, 0.0, -1.0])

        print("=" * 60)
        print("Arm Controller Ready")
        print("=" * 60)


    def _print_joint_bounds(self):
        """관절 한계값 출력"""
        print("=" * 60)
        print("Joint Bounds from URDF:")
        print("=" * 60)
        for i, bounds in self.joint_bounds.items():
            if bounds is not None:
                link_name = self.chain.links[i].name
                lower_deg = np.degrees(bounds[0])
                upper_deg = np.degrees(bounds[1])
                print(
                    f"Joint {i} ({link_name}): "
                    f"[{bounds[0]:.5f}, {bounds[1]:.5f}] rad "
                    f"= [{lower_deg:.1f}°, {upper_deg:.1f}°]"
                )
        print("=" * 60)


    def clip_joint(self, joint_idx: int, value: float) -> float:
        """관절 한계에 따라 값을 클립"""
        if joint_idx not in self.joint_bounds or self.joint_bounds[joint_idx] is None:
            return value
        
        low, high = self.joint_bounds[joint_idx]
        clipped = np.clip(value, low, high)
        
        if abs(clipped - value) > 0.001:
            link_name = self.chain.links[joint_idx].name if joint_idx < len(self.chain.links) else "Unknown"
            clipped_deg = np.degrees(clipped)
            original_deg = np.degrees(value)
            print(
                f"⚠️  Joint {joint_idx} ({link_name}) OUT OF BOUNDS: "
                f"{original_deg:.1f}° → {clipped_deg:.1f}° "
                f"(bounds: [{np.degrees(low):.1f}°, {np.degrees(high):.1f}°])"
            )
        
        return clipped


    def validate_joint_config(self, joints_array: np.ndarray) -> Tuple[bool, Optional[str]]:
        """FK로 관절 배열 검증"""
        try:
            self.chain.forward_kinematics(joints_array)
            return True, None
        except Exception as e:
            return False, f"FK computation failed: {str(e)}"


    def step(self, dt: float = 0.02):
        """보간된 관절값 업데이트"""
        if self.is_moving:
            now = time.time()
            elapsed = now - self.move_start_time
            
            if self.move_duration <= 0:
                progress = 1.0
            else:
                progress = min(elapsed / self.move_duration, 1.0)
            
            self.current_ik_joints = (self.start_ik_joints * (1 - progress)) + (self.target_ik_joints * progress)
            
            if progress >= 1.0:
                self.is_moving = False


    def get_joint_state(self) -> dict:
        """현재 관절 상태 반환"""
        pan = float(self.current_ik_joints[1])
        lift = float(self.current_ik_joints[2])
        elbow = float(self.current_ik_joints[3])
        
        if self.manual_flex_override is not None:
            flex = float(self.manual_flex_override)
        else:
            flex = float(self.current_ik_joints[4])
        
        roll = float(self.current_wrist_roll)
        grip = float(self.current_gripper)
        
        return {
            'pan': pan,
            'lift': lift,
            'elbow': elbow,
            'flex': flex,
            'roll': roll,
            'gripper': grip,
            'speed': self.speed,
            'is_moving': self.is_moving
        }


    def get_forward_kinematics(self) -> np.ndarray:
        """현재 관절 상태의 FK 계산"""
        current_joints_combined = np.array(self.current_ik_joints, dtype=float).copy()
        if self.manual_flex_override is not None:
            current_joints_combined[4] = float(self.manual_flex_override)
        
        return self.chain.forward_kinematics(current_joints_combined)


    def move_to_xyz(self, x: float, y: float, z: float, direction: Optional[np.ndarray] = None, duration: Optional[float] = None):
        """지정된 위치로 팔 이동"""
        target_pos = np.array([x, y, z], dtype=float)
        
        if direction is None:
            current_joints_combined = np.array(self.current_ik_joints, dtype=float).copy()
            if self.manual_flex_override is not None:
                current_joints_combined[4] = float(self.manual_flex_override)
            
            current_fk = self.chain.forward_kinematics(current_joints_combined)
            self.target_dir = current_fk[:3, 2]
            print(f"🎯 IK Goal: pos={target_pos}, Keep Dir: {self.target_dir}")
        else:
            self.target_dir = np.array(direction, dtype=float)
            print(f"🎯 IK Goal: pos={target_pos}, New Dir: {self.target_dir}")

        try:
            ik_start_guess = self.current_ik_joints.copy()

            ik_solution = self.chain.inverse_kinematics(
                target_position=target_pos,
                target_orientation=self.target_dir,
                orientation_mode="Z",
                initial_position=ik_start_guess
            )
            
            for i in range(len(ik_solution)):
                if self.chain.links[i].joint_type == 'revolute':
                    low = self.chain.links[i].bounds[0]
                    high = self.chain.links[i].bounds[1]
                    ik_solution[i] = np.clip(ik_solution[i], low, high)
            
            fk_res = self.chain.forward_kinematics(ik_solution)
            real_pos = fk_res[:3, 3]
            real_orient_z = fk_res[:3, 2]
            
            pos_error = np.linalg.norm(target_pos - real_pos)
            
            target_dir_norm = self.target_dir / np.linalg.norm(self.target_dir)
            real_orient_z_norm = real_orient_z / np.linalg.norm(real_orient_z)
            dot_prod = np.dot(target_dir_norm, real_orient_z_norm)
            angle_error_deg = np.degrees(np.arccos(np.clip(dot_prod, -1.0, 1.0)))

            print(f"  >> IK Validation: PosErr={pos_error:.4f}m, AngErr={angle_error_deg:.1f}°")

            if pos_error > 0.02 or angle_error_deg > 10.0:
                print(f"❌ UNREACHABLE!")
                return

            self.start_ik_joints = self.current_ik_joints.copy()
            self.target_ik_joints = ik_solution
            self.manual_flex_override = None
            
            if duration is None:
                max_diff = np.max(np.abs(self.target_ik_joints - self.start_ik_joints))
                self.move_duration = max(max_diff * 1.0, 1.5)
            else:
                self.move_duration = duration
            
            self.move_start_time = time.time()
            self.is_moving = True
            
            print(f"  >> Motion started... Duration: {self.move_duration:.2f}s")

        except Exception as e:
            print(f"❌ IK Failed: {e}")


    def check_and_control(self, joint_idx: int, val: float):
        """개별 관절 값 설정 및 검증"""
        if joint_idx == 4:
            ctrl_name = "flex"
        elif joint_idx == 5:
            ctrl_name = "roll"
        elif joint_idx == 6:
            ctrl_name = "gripper"
        else:
            print(f"❌ Invalid joint index: {joint_idx}")
            return

        val_clipped = self.clip_joint(joint_idx, val)
        
        test_joints = np.array(self.current_ik_joints, dtype=float).copy()
        test_joints[joint_idx] = val_clipped
        
        is_valid, error_msg = self.validate_joint_config(test_joints)
        
        if not is_valid:
            print(f"❌ {ctrl_name} value REJECTED: {error_msg}")
            return
        
        if self.is_moving:
            self.is_moving = False
            print(f"⚙️  Interrupted IK motion → Manual {ctrl_name} mode")
        
        val_deg = np.degrees(val_clipped)
        self.current_ik_joints[joint_idx] = val_clipped
        if joint_idx == 4:
            self.manual_flex_override = val_clipped
        elif joint_idx == 5:
            self.current_wrist_roll = val_clipped
        elif joint_idx == 6:
            self.current_gripper = val_clipped
        print(f"✅ Manual {ctrl_name}: {val_clipped:.4f} rad ({val_deg:.1f}°)")


    def set_flex(self, angle_rad: float):
        """손목 굴곡(flex) 설정"""
        self.check_and_control(4, angle_rad)

    def set_roll(self, angle_rad: float):
        """손목 회전(roll) 설정"""
        self.check_and_control(5, angle_rad)

    def set_gripper(self, angle_rad: float):
        """그리퍼 설정"""
        self.check_and_control(6, angle_rad)

    def set_speed(self, speed: float):
        """팔 이동 속도 배수 설정"""
        self.speed = max(0.1, speed)
        print(f"Speed set to: {self.speed}")

    def find_nearby_reachable_position(self, target_x: float, target_y: float, target_z: float, 
                                   search_radius: float = 0.05, 
                                   grid_resolution: int = 5) -> Optional[np.ndarray]:
        """
        목표 좌표 근처에서 도달 가능한 좌표를 찾음
        
        Args:
            target_x, target_y, target_z: 원래 목표 좌표
            search_radius: 검색 반경 (m)
            grid_resolution: 그리드 밀도 (높을수록 세밀함)
        
        Returns:
            도달 가능한 좌표 [x, y, z] 또는 None
        """
        target_pos = np.array([target_x, target_y, target_z])
        best_pos = None
        best_distance = float('inf')
        
        # 그리드 생성
        step = search_radius / grid_resolution
        search_points = []
        
        for dx in np.linspace(-search_radius, search_radius, grid_resolution):
            for dy in np.linspace(-search_radius, search_radius, grid_resolution):
                for dz in np.linspace(-search_radius, search_radius, grid_resolution):
                    test_pos = target_pos + np.array([dx, dy, dz])
                    search_points.append(test_pos)
        
        print(f"\n🔍 Searching {len(search_points)} candidate positions...")
        found_count = 0
        
        for test_pos in search_points:
            try:
                # IK 시도
                ik_solution = self.chain.inverse_kinematics(
                    target_position=test_pos,
                    target_orientation=self.target_dir,
                    orientation_mode="Z",
                    initial_position=self.current_ik_joints.copy()
                )
                
                # 경계 클립
                for i in range(len(ik_solution)):
                    if self.chain.links[i].joint_type == 'revolute':
                        low, high = self.chain.links[i].bounds
                        ik_solution[i] = np.clip(ik_solution[i], low, high)
                
                # FK 검증
                fk_result = self.chain.forward_kinematics(ik_solution)
                computed_pos = fk_result[:3, 3]
                error = np.linalg.norm(computed_pos - test_pos)
                
                if error < 0.02:  # 2cm 이내면 성공
                    found_count += 1
                    distance_to_target = np.linalg.norm(test_pos - target_pos)
                    
                    if distance_to_target < best_distance:
                        best_distance = distance_to_target
                        best_pos = test_pos
                        
                        print(f"  ✅ Found: ({test_pos[0]:.3f}, {test_pos[1]:.3f}, {test_pos[2]:.3f})")
            
            except:
                pass
        
        print(f"\n📊 Search complete: {found_count} reachable positions found")
        
        if best_pos is not None:
            print(f"\n🎯 BEST MATCH (closest to target):")
            print(f"   Target:  ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
            print(f"   Recommended: ({best_pos[0]:.3f}, {best_pos[1]:.3f}, {best_pos[2]:.3f})")
            print(f"   Distance: {best_distance:.4f}m")
            return best_pos
        else:
            print(f"❌ No reachable position found within {search_radius}m of target")
            return None
