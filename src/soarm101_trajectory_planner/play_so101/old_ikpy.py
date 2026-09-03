# #잘 됨 ver.1
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import ikpy.chain
# import numpy as np

# class ArmControllerNode(Node):
#     def __init__(self):
#         super().__init__('arm_controller_node')

#         urdf_path = "/home/csh/Desktop/dt_ws/src/dt_arm_description/urdf/so101_new_calib.urdf"
        
#         # 4DOF IK: Pan, Lift, Elbow, Flex만 IK로 계산 (Roll, Gripper 제외)
#         # IKPy는 앞의 4개만 건드림
#         active_mask = [False, True, True, True, True, False, False]
        
#         self.chain = ikpy.chain.Chain.from_urdf_file(
#             urdf_path,
#             base_elements=["base_link"],
#             active_links_mask=active_mask
#         )

#         # --- 로봇의 현재 상태 저장소 ---
#         # IK로 계산된 관절값 (Pan, Lift, Elbow, Flex) 초기화
#         self.current_ik_joints = np.zeros(len(self.chain.links)) 
        
#         # 개별 제어 관절값 초기화
#         self.manual_flex_override = None # 사용자가 4번 모터를 수동으로 건드렸는지 여부
#         self.current_wrist_roll = 0.0    # 5번 모터
#         self.current_gripper = 0.0       # 6번 모터
#         self.speed = 1.0
#         self.target_dir = np.array([0.0, 0.0, -1.0])

#         # --- Publishers ---
#         self.publisher_ = self.create_publisher(Float64MultiArray, '/dt_arm/joint_cmd', 10)

#         # --- Subscribers ---
        
#         # 1. IK 목표 + 방향 ([x, y, z] 또는 [x, y, z, dx, dy, dz])
#         self.sub_ik = self.create_subscription(
#             Float64MultiArray, '/dt_arm/ik_target_xyz', self.ik_callback, 10)

#         # 2. 4번 모터(Flex) 수동 제어
#         self.sub_flex = self.create_subscription(
#             Float64MultiArray, '/dt_arm/wrist_flex_cmd', self.flex_callback, 10)

#         # 3. 5번 모터(Roll) 수동 제어
#         self.sub_roll = self.create_subscription(
#             Float64MultiArray, '/dt_arm/wrist_roll_cmd', self.roll_callback, 10)

#         # 4. 6번 모터(Gripper) 수동 제어
#         self.sub_grip = self.create_subscription(
#             Float64MultiArray, '/dt_arm/gripper_cmd', self.gripper_callback, 10)
        
#         print(f"Arm Controller Ready!")
#         print(f" - IK Control (Pos+Dir): /dt_arm/ik_target_xyz")
#         print(f" - Manual Joints: /dt_arm/wrist_flex_cmd, .../wrist_roll_cmd, .../gripper_cmd")


#     def publish_joints(self):
#         """현재 저장된 상태들을 조합해서 최종 명령 전송"""
        
#         # IKPy 결과에서 1~4번 관절 추출
#         # ik_solution 구조: [Base, Pan, Lift, Elbow, Flex, Roll, Gripper]
#         pan = float(self.current_ik_joints[1])
#         lift = float(self.current_ik_joints[2])
#         elbow = float(self.current_ik_joints[3])
        
#         # 4번 모터(Flex): 수동 오버라이드 값이 있으면 그거 쓰고, 아니면 IK 값 사용
#         if self.manual_flex_override is not None:
#             flex = float(self.manual_flex_override)
#         else:
#             flex = float(self.current_ik_joints[4])

#         # 5번(Roll), 6번(Gripper)은 저장된 값 사용
#         roll = float(self.current_wrist_roll)
#         grip = float(self.current_gripper)

#         # 최종 패킷 생성
#         final_cmd = [pan, lift, elbow, flex, roll, grip, self.speed]
        
#         msg = Float64MultiArray()
#         msg.data = final_cmd
#         self.publisher_.publish(msg)
        
#         # 디버깅 출력
#         # print(f"Pub: {np.round(final_cmd[:6], 3)}")


#     def ik_callback(self, msg):
#         """
#         입력 데이터 형식:
#         1. [x, y, z] -> 기본 방향(수직 하단)으로 이동
#         2. [x, y, z, dx, dy, dz] -> 지정된 벡터(dx,dy,dz)를 바라보며 이동
#         """
#         data = msg.data
#         target_pos = np.array(data[:3], dtype=float)
        
#         # 방향 벡터 설정
#         if len(data) >= 6:
#             self.target_dir = np.array(data[3:6], dtype=float)
#             print(f"IK Goal: {target_pos}, Dir: {self.target_dir}")
#         else:
#             current_joints_combined = np.array(self.current_ik_joints, dtype=float).copy()
#             if self.manual_flex_override is not None:
#                 current_joints_combined[4] = float(self.manual_flex_override)
#             print(f"DEBUG: Joints for FK: {current_joints_combined}, Type: {type(current_joints_combined)}")

#             current_fk = self.chain.forward_kinematics(current_joints_combined)
            
#             self.target_dir = current_fk[:3, 2]
#             print(f"IK Goal: {target_pos}, Dir: {self.target_dir}")

#         try:
#             # IK 계산
#             ik_solution = self.chain.inverse_kinematics(
#                 target_position=target_pos,
#                 target_orientation=self.target_dir,
#                 orientation_mode="Z",  # TCP의 Z축을 target_dir에 맞춤
#                 initial_position=self.current_ik_joints
#             )
            
#             # 관절 한계 적용
#             for i in range(len(ik_solution)):
#                 if self.chain.links[i].joint_type == 'revolute':
#                     low = self.chain.links[i].bounds[0]
#                     high = self.chain.links[i].bounds[1]
#                     ik_solution[i] = np.clip(ik_solution[i], low, high)
            

#             # [추가] 2. IK 솔루션 검증 (Validation)
#             # 계산된 관절각으로 실제로 어디에 가는지 FK로 확인
#             fk_res = self.chain.forward_kinematics(ik_solution)
#             real_pos = fk_res[:3, 3]
#             real_orient_z = fk_res[:3, 2]  # TCP의 Z축 벡터
            
#             # (1) 위치 오차 계산
#             pos_error = np.linalg.norm(target_pos - real_pos)
            
#             # (2) 방향 오차 계산 (목표 벡터와 실제 벡터의 내적 -> 각도 차이)
#             # 내적이 1이면 일치, -1이면 반대, 0이면 직각
#             # 안전하게 정규화 후 계산
#             target_dir_norm = self.target_dir / np.linalg.norm(self.target_dir)
#             real_orient_z_norm = real_orient_z / np.linalg.norm(real_orient_z)
#             dot_prod = np.dot(target_dir_norm, real_orient_z_norm)
#             angle_error_deg = np.degrees(np.arccos(np.clip(dot_prod, -1.0, 1.0)))

#             print(f" >> IK Check: PosErr={pos_error:.4f}m, AngErr={angle_error_deg:.1f}deg")

#             # 허용 오차 기준 (예: 위치 2cm, 각도 10도)
#             if pos_error > 0.02 or angle_error_deg > 10.0:
#                 self.get_logger().warn(f"⚠️ UNREACHABLE! Too far from goal.")
#                 self.get_logger().warn(f"   Goal: {target_pos} Dir: {self.target_dir}")
#                 self.get_logger().warn(f"   Real: {real_pos} Dir: {real_orient_z}")
#                 return  # <--- [중요] 명령 취소! 로봇 안 움직임
#             # ---------------------------------------------------------
            
#             # 검증 통과하면 업데이트
#             self.current_ik_joints = ik_solution
#             self.manual_flex_override = None 
#             self.publish_joints()

#         except Exception as e:
#             self.get_logger().error(f"IK Failed: {e}")



#     def flex_callback(self, msg):
#         """4번 모터(Flex) 수동 제어"""
#         val = msg.data[0]
#         self.manual_flex_override = val # 수동 모드 활성화
#         self.publish_joints()
#         print(f"Manual Flex: {val}")

#     def roll_callback(self, msg):
#         """5번 모터(Roll) 수동 제어"""
#         self.current_wrist_roll = msg.data[0]
#         self.publish_joints()
#         print(f"Manual Roll: {self.current_wrist_roll}")

#     def gripper_callback(self, msg):
#         """6번 모터(Gripper) 수동 제어"""
#         self.current_gripper = msg.data[0]
#         self.publish_joints()
#         print(f"Manual Gripper: {self.current_gripper}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmControllerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #잘 됨 ver.2
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import ikpy.chain
# import numpy as np
# import time

# class ArmControllerNode(Node):
#     def __init__(self):
#         super().__init__('arm_controller_node')

#         urdf_path = "/home/csh/Desktop/dt_ws/src/dt_arm_description/urdf/so101_new_calib.urdf"
        
#         # 4DOF IK: Pan, Lift, Elbow, Flex만 IK로 계산 (Roll, Gripper 제외)
#         # IKPy는 앞의 4개만 건드림
#         active_mask = [False, True, True, True, True, False, False]
        
#         self.chain = ikpy.chain.Chain.from_urdf_file(
#             urdf_path,
#             base_elements=["base_link"],
#             active_links_mask=active_mask
#         )

#         # --- 로봇의 현재 상태 저장소 ---
#         # self.current_ik_joints: 현재 실제로 로봇이 위치하고 있는(혹은 가고 있는 중간) 관절값
#         self.current_ik_joints = np.zeros(len(self.chain.links)) 
        
#         # 보간(Trajectory) 이동을 위한 변수들
#         self.start_ik_joints = np.zeros(len(self.chain.links))  # 이동 시작 지점
#         self.target_ik_joints = np.zeros(len(self.chain.links)) # 최종 목표 지점
#         self.is_moving = False
#         self.move_start_time = 0.0
#         self.move_duration = 2.0  # 기본 이동 시간 (초)

#         # 개별 제어 관절값 초기화
#         self.manual_flex_override = None # 사용자가 4번 모터를 수동으로 건드렸는지 여부
#         self.current_wrist_roll = 0.0    # 5번 모터
#         self.current_gripper = 0.0       # 6번 모터
#         self.speed = 1.0
#         self.target_dir = np.array([0.0, 0.0, -1.0])

#         # --- Publishers ---
#         self.publisher_ = self.create_publisher(Float64MultiArray, '/dt_arm/joint_cmd', 10)

#         # --- Subscribers ---
#         # 1. IK 목표 + 방향 ([x, y, z] 또는 [x, y, z, dx, dy, dz])
#         self.sub_ik = self.create_subscription(
#             Float64MultiArray, '/dt_arm/ik_target_xyz', self.ik_callback, 10)

#         # 2. 4번 모터(Flex) 수동 제어
#         self.sub_flex = self.create_subscription(
#             Float64MultiArray, '/dt_arm/wrist_flex_cmd', self.flex_callback, 10)

#         # 3. 5번 모터(Roll) 수동 제어
#         self.sub_roll = self.create_subscription(
#             Float64MultiArray, '/dt_arm/wrist_roll_cmd', self.roll_callback, 10)

#         # 4. 6번 모터(Gripper) 수동 제어
#         self.sub_grip = self.create_subscription(
#             Float64MultiArray, '/dt_arm/gripper_cmd', self.gripper_callback, 10)
        
#         # --- Timer (Trajectory Loop) ---
#         # 50Hz (0.02초) 주기로 실행되어 관절을 부드럽게 움직임
#         self.timer = self.create_timer(0.02, self.control_loop)

#         print(f"Arm Controller Ready with Trajectory Smoothing!")
#         print(f" - IK Control (Pos+Dir): /dt_arm/ik_target_xyz")
#         print(f" - Manual Joints: /dt_arm/wrist_flex_cmd, .../wrist_roll_cmd, .../gripper_cmd")

#     def control_loop(self):
#         """주기적으로 실행되며 보간된 관절값을 계산해 전송"""
#         if self.is_moving:
#             now = time.time()
#             elapsed = now - self.move_start_time
            
#             # 진행률 계산 (0.0 ~ 1.0)
#             if self.move_duration <= 0:
#                 progress = 1.0
#             else:
#                 progress = min(elapsed / self.move_duration, 1.0)
            
#             # [핵심] 선형 보간 (Lerp): start에서 target까지 progress 비율만큼 이동
#             # 공식: current = start * (1-p) + target * p
#             self.current_ik_joints = (self.start_ik_joints * (1 - progress)) + (self.target_ik_joints * progress)
            
#             # 목표 도달 확인
#             if progress >= 1.0:
#                 self.is_moving = False
#                 # print("Motion Complete.")
        
#         # 현재 계산된 위치(보간 중이든 아니든)를 전송
#         self.publish_joints()

#     def publish_joints(self):
#         """현재 저장된 상태들을 조합해서 최종 명령 전송"""
        
#         # IKPy 결과에서 1~4번 관절 추출
#         # ik_solution 구조: [Base, Pan, Lift, Elbow, Flex, Roll, Gripper]
#         pan = float(self.current_ik_joints[1])
#         lift = float(self.current_ik_joints[2])
#         elbow = float(self.current_ik_joints[3])
        
#         # 4번 모터(Flex): 수동 오버라이드 값이 있으면 그거 쓰고, 아니면 IK 값 사용
#         if self.manual_flex_override is not None:
#             flex = float(self.manual_flex_override)
#         else:
#             flex = float(self.current_ik_joints[4])

#         # 5번(Roll), 6번(Gripper)은 저장된 값 사용
#         roll = float(self.current_wrist_roll)
#         grip = float(self.current_gripper)

#         # 최종 패킷 생성
#         final_cmd = [pan, lift, elbow, flex, roll, grip, self.speed]
        
#         msg = Float64MultiArray()
#         msg.data = final_cmd
#         self.publisher_.publish(msg)

#     def ik_callback(self, msg):
#         """
#         입력 데이터 형식:
#         1. [x, y, z] -> 기본 방향(수직 하단)으로 이동
#         2. [x, y, z, dx, dy, dz] -> 지정된 벡터(dx,dy,dz)를 바라보며 이동
#         """
#         data = msg.data
#         target_pos = np.array(data[:3], dtype=float)
        
#         # 1. 방향 설정 로직 (수동 Flex 반영)
#         if len(data) >= 6:
#             self.target_dir = np.array(data[3:6], dtype=float)
#             print(f"New IK Goal: {target_pos}, New Dir: {self.target_dir}")
#         else:
#             # 현재 관절 상태 안전하게 가져오기 (리스트/배열 타입 보장)
#             current_joints_combined = np.array(self.current_ik_joints, dtype=float).copy()
#             if self.manual_flex_override is not None:
#                 current_joints_combined[4] = float(self.manual_flex_override)
            
#             # print(f"DEBUG: Joints for FK: {current_joints_combined}")

#             current_fk = self.chain.forward_kinematics(current_joints_combined)
            
#             self.target_dir = current_fk[:3, 2]
#             print(f"New IK Goal: {target_pos}, Keep Dir: {self.target_dir}")

#         try:
#             # IK 계산 시작점 설정 (현재 위치에서 시작해야 함)
#             # 만약 이동 중이었다면 현재 보간 중인 위치를 시작점으로 사용
#             ik_start_guess = self.current_ik_joints.copy()

#             # IK 계산
#             ik_solution = self.chain.inverse_kinematics(
#                 target_position=target_pos,
#                 target_orientation=self.target_dir,
#                 orientation_mode="Z",  # TCP의 Z축을 target_dir에 맞춤
#                 initial_position=ik_start_guess
#             )
            
#             # 관절 한계 적용
#             for i in range(len(ik_solution)):
#                 if self.chain.links[i].joint_type == 'revolute':
#                     low = self.chain.links[i].bounds[0]
#                     high = self.chain.links[i].bounds[1]
#                     ik_solution[i] = np.clip(ik_solution[i], low, high)
            
#             # [검증] IK 솔루션 검증 (Validation)
#             fk_res = self.chain.forward_kinematics(ik_solution)
#             real_pos = fk_res[:3, 3]
#             real_orient_z = fk_res[:3, 2]  # TCP의 Z축 벡터
            
#             # (1) 위치 오차 계산
#             pos_error = np.linalg.norm(target_pos - real_pos)
            
#             # (2) 방향 오차 계산
#             target_dir_norm = self.target_dir / np.linalg.norm(self.target_dir)
#             real_orient_z_norm = real_orient_z / np.linalg.norm(real_orient_z)
#             dot_prod = np.dot(target_dir_norm, real_orient_z_norm)
#             angle_error_deg = np.degrees(np.arccos(np.clip(dot_prod, -1.0, 1.0)))

#             print(f" >> IK Check: PosErr={pos_error:.4f}m, AngErr={angle_error_deg:.1f}deg")

#             # 허용 오차 기준 (예: 위치 2cm, 각도 10도)
#             if pos_error > 0.02 or angle_error_deg > 10.0:
#                 self.get_logger().warn(f"⚠️ UNREACHABLE! Too far from goal.")
#                 return  # 명령 취소

#             # 검증 통과 -> 궤적 생성 시작 (Trajectory Setup)
#             self.start_ik_joints = self.current_ik_joints.copy() # 현재 위치 저장
#             self.target_ik_joints = ik_solution                 # 목표 위치 저장
            
#             # Flex 모터 수동 오버라이드 해제 (IK 결과값인 target_ik_joints[4]로 자연스럽게 이동하기 위해)
#             # 단, 이 부분은 정책에 따라 다를 수 있음. IK가 Flex까지 제어하게 하려면 None으로 초기화.
#             self.manual_flex_override = None 
            
#             # 이동 시간 자동 계산 (관절 변화량이 클수록 천천히)
#             # 최대 변화각(라디안) * 계수 = 시간(초)
#             max_diff = np.max(np.abs(self.target_ik_joints - self.start_ik_joints))
#             self.move_duration = max(max_diff * 3.0, 1.5) # 최소 1.5초는 보장 (부드럽게)
            
#             self.move_start_time = time.time()
#             self.is_moving = True
            
#             print(f" >> Moving Start... Duration: {self.move_duration:.2f}s")

#         except Exception as e:
#             self.get_logger().error(f"IK Failed: {e}")

#     def flex_callback(self, msg):
#         """4번 모터(Flex) 수동 제어"""
#         val = msg.data[0]
#         self.manual_flex_override = val # 수동 모드 활성화
#         # 수동 제어는 즉각 반응해야 하므로 여기서 바로 publish 할 수도 있지만, 
#         # Control Loop가 0.02초마다 돌고 있으므로 값만 바꾸면 바로 반영됨.
#         print(f"Manual Flex: {val}")

#     def roll_callback(self, msg):
#         """5번 모터(Roll) 수동 제어"""
#         self.current_wrist_roll = msg.data[0]
#         print(f"Manual Roll: {self.current_wrist_roll}")

#     def gripper_callback(self, msg):
#         """6번 모터(Gripper) 수동 제어"""
#         self.current_gripper = msg.data[0]
#         print(f"Manual Gripper: {self.current_gripper}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArmControllerNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()