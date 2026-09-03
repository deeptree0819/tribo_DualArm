#!/usr/bin/env python3
"""
gripper_pose_gui.py: 그리퍼의 실시간 XYZ 좌표를 GUI 창에 표시합니다.

사용법:
    ros2 run soarm101_trajectory_planner gripper_pose_gui.py
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import TransformListener, Buffer
from sensor_msgs.msg import JointState

import math
import sys
import threading

# PyQt5 사용 시도, 없으면 tkinter 사용
try:
    from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                                  QLabel, QFrame, QGroupBox, QGridLayout)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt5.QtGui import QFont
    USE_PYQT = True
except ImportError:
    USE_PYQT = False
    import tkinter as tk
    from tkinter import ttk


def quaternion_to_rpy(x, y, z, w):
    """Quaternion을 Roll, Pitch, Yaw로 변환"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PoseData:
    """좌표 데이터 저장"""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.joints = {}


class ROSNode(Node):
    """ROS2 노드"""
    def __init__(self, pose_data: PoseData):
        super().__init__('gripper_pose_gui')
        
        self.pose_data = pose_data
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.base_frame = 'base_link'
        self.gripper_frame = 'gripper_link'
        
        # Joint state 구독
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        # 타이머
        self.timer = self.create_timer(0.05, self.update_pose)  # 20Hz
        
    def joint_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.pose_data.joints[name] = msg.position[i]
    
    def update_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.gripper_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            
            pos = transform.transform.translation
            rot = transform.transform.rotation
            
            self.pose_data.x = pos.x
            self.pose_data.y = pos.y
            self.pose_data.z = pos.z
            
            roll, pitch, yaw = quaternion_to_rpy(rot.x, rot.y, rot.z, rot.w)
            self.pose_data.roll = roll
            self.pose_data.pitch = pitch
            self.pose_data.yaw = yaw
            
        except Exception:
            pass


if USE_PYQT:
    class PyQtGUI(QWidget):
        """PyQt5 GUI"""
        def __init__(self, pose_data: PoseData):
            super().__init__()
            self.pose_data = pose_data
            self.init_ui()
            
            # 업데이트 타이머
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_display)
            self.timer.start(50)  # 20Hz
        
        def init_ui(self):
            self.setWindowTitle('SO-ARM101 Gripper Pose Monitor')
            self.setMinimumWidth(400)
            self.setStyleSheet("""
                QWidget { background-color: #2b2b2b; color: #ffffff; }
                QGroupBox { 
                    border: 2px solid #555; 
                    border-radius: 5px; 
                    margin-top: 10px;
                    font-weight: bold;
                }
                QGroupBox::title { 
                    subcontrol-origin: margin; 
                    left: 10px; 
                    padding: 0 5px;
                }
                QLabel { font-size: 14px; }
            """)
            
            layout = QVBoxLayout()
            
            # 제목
            title = QLabel('🤖 Gripper Position')
            title.setFont(QFont('Arial', 16, QFont.Bold))
            title.setAlignment(Qt.AlignCenter)
            layout.addWidget(title)
            
            # Position Group
            pos_group = QGroupBox('Position')
            pos_layout = QGridLayout()
            
            # X
            pos_layout.addWidget(QLabel('X:'), 0, 0)
            self.x_label = QLabel('0.0000 m')
            self.x_label.setStyleSheet('color: #ff6b6b; font-weight: bold; font-size: 16px;')
            pos_layout.addWidget(self.x_label, 0, 1)
            self.x_mm_label = QLabel('(0.0 mm)')
            self.x_mm_label.setStyleSheet('color: #888;')
            pos_layout.addWidget(self.x_mm_label, 0, 2)
            
            # Y
            pos_layout.addWidget(QLabel('Y:'), 1, 0)
            self.y_label = QLabel('0.0000 m')
            self.y_label.setStyleSheet('color: #4ecdc4; font-weight: bold; font-size: 16px;')
            pos_layout.addWidget(self.y_label, 1, 1)
            self.y_mm_label = QLabel('(0.0 mm)')
            self.y_mm_label.setStyleSheet('color: #888;')
            pos_layout.addWidget(self.y_mm_label, 1, 2)
            
            # Z
            pos_layout.addWidget(QLabel('Z:'), 2, 0)
            self.z_label = QLabel('0.0000 m')
            self.z_label.setStyleSheet('color: #45b7d1; font-weight: bold; font-size: 16px;')
            pos_layout.addWidget(self.z_label, 2, 1)
            self.z_mm_label = QLabel('(0.0 mm)')
            self.z_mm_label.setStyleSheet('color: #888;')
            pos_layout.addWidget(self.z_mm_label, 2, 2)
            
            pos_group.setLayout(pos_layout)
            layout.addWidget(pos_group)
            
            # Orientation Group
            rot_group = QGroupBox('Orientation')
            rot_layout = QGridLayout()
            
            rot_layout.addWidget(QLabel('Roll:'), 0, 0)
            self.roll_label = QLabel('0.0°')
            self.roll_label.setStyleSheet('color: #f9ca24; font-size: 14px;')
            rot_layout.addWidget(self.roll_label, 0, 1)
            
            rot_layout.addWidget(QLabel('Pitch:'), 1, 0)
            self.pitch_label = QLabel('0.0°')
            self.pitch_label.setStyleSheet('color: #f9ca24; font-size: 14px;')
            rot_layout.addWidget(self.pitch_label, 1, 1)
            
            rot_layout.addWidget(QLabel('Yaw:'), 2, 0)
            self.yaw_label = QLabel('0.0°')
            self.yaw_label.setStyleSheet('color: #f9ca24; font-size: 14px;')
            rot_layout.addWidget(self.yaw_label, 2, 1)
            
            rot_group.setLayout(rot_layout)
            layout.addWidget(rot_group)
            
            # Joint Angles Group
            joint_group = QGroupBox('Joint Angles')
            self.joint_layout = QGridLayout()
            
            self.joint_labels = {}
            joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
            for i, name in enumerate(joint_names):
                short_name = name.replace('_', ' ').title()
                self.joint_layout.addWidget(QLabel(f'{short_name}:'), i, 0)
                label = QLabel('0.0°')
                label.setStyleSheet('color: #a29bfe; font-size: 12px;')
                self.joint_layout.addWidget(label, i, 1)
                self.joint_labels[name] = label
            
            joint_group.setLayout(self.joint_layout)
            layout.addWidget(joint_group)
            
            self.setLayout(layout)
        
        def update_display(self):
            # Position
            self.x_label.setText(f'{self.pose_data.x:+.4f} m')
            self.y_label.setText(f'{self.pose_data.y:+.4f} m')
            self.z_label.setText(f'{self.pose_data.z:+.4f} m')
            
            self.x_mm_label.setText(f'({self.pose_data.x*1000:+.1f} mm)')
            self.y_mm_label.setText(f'({self.pose_data.y*1000:+.1f} mm)')
            self.z_mm_label.setText(f'({self.pose_data.z*1000:+.1f} mm)')
            
            # Orientation
            self.roll_label.setText(f'{math.degrees(self.pose_data.roll):+.1f}°')
            self.pitch_label.setText(f'{math.degrees(self.pose_data.pitch):+.1f}°')
            self.yaw_label.setText(f'{math.degrees(self.pose_data.yaw):+.1f}°')
            
            # Joints
            for name, label in self.joint_labels.items():
                if name in self.pose_data.joints:
                    deg = math.degrees(self.pose_data.joints[name])
                    label.setText(f'{deg:+.1f}°')

else:
    class TkinterGUI:
        """Tkinter GUI (PyQt5 없을 때)"""
        def __init__(self, pose_data: PoseData):
            self.pose_data = pose_data
            self.root = tk.Tk()
            self.root.title('SO-ARM101 Gripper Pose Monitor')
            self.root.configure(bg='#2b2b2b')
            
            self.init_ui()
            
        def init_ui(self):
            # Title
            title = tk.Label(self.root, text='🤖 Gripper Position', 
                           font=('Arial', 16, 'bold'), bg='#2b2b2b', fg='white')
            title.pack(pady=10)
            
            # Position Frame
            pos_frame = ttk.LabelFrame(self.root, text='Position')
            pos_frame.pack(padx=10, pady=5, fill='x')
            
            self.x_var = tk.StringVar(value='X: 0.0000 m')
            self.y_var = tk.StringVar(value='Y: 0.0000 m')
            self.z_var = tk.StringVar(value='Z: 0.0000 m')
            
            tk.Label(pos_frame, textvariable=self.x_var, font=('Courier', 12), 
                    fg='#ff6b6b', bg='#2b2b2b').pack(anchor='w', padx=5)
            tk.Label(pos_frame, textvariable=self.y_var, font=('Courier', 12),
                    fg='#4ecdc4', bg='#2b2b2b').pack(anchor='w', padx=5)
            tk.Label(pos_frame, textvariable=self.z_var, font=('Courier', 12),
                    fg='#45b7d1', bg='#2b2b2b').pack(anchor='w', padx=5)
            
            # Rotation Frame
            rot_frame = ttk.LabelFrame(self.root, text='Orientation')
            rot_frame.pack(padx=10, pady=5, fill='x')
            
            self.roll_var = tk.StringVar(value='Roll: 0.0°')
            self.pitch_var = tk.StringVar(value='Pitch: 0.0°')
            self.yaw_var = tk.StringVar(value='Yaw: 0.0°')
            
            tk.Label(rot_frame, textvariable=self.roll_var, font=('Courier', 10),
                    fg='#f9ca24', bg='#2b2b2b').pack(anchor='w', padx=5)
            tk.Label(rot_frame, textvariable=self.pitch_var, font=('Courier', 10),
                    fg='#f9ca24', bg='#2b2b2b').pack(anchor='w', padx=5)
            tk.Label(rot_frame, textvariable=self.yaw_var, font=('Courier', 10),
                    fg='#f9ca24', bg='#2b2b2b').pack(anchor='w', padx=5)
            
            self.root.after(50, self.update_display)
        
        def update_display(self):
            self.x_var.set(f'X: {self.pose_data.x:+.4f} m  ({self.pose_data.x*1000:+.1f} mm)')
            self.y_var.set(f'Y: {self.pose_data.y:+.4f} m  ({self.pose_data.y*1000:+.1f} mm)')
            self.z_var.set(f'Z: {self.pose_data.z:+.4f} m  ({self.pose_data.z*1000:+.1f} mm)')
            
            self.roll_var.set(f'Roll:  {math.degrees(self.pose_data.roll):+.1f}°')
            self.pitch_var.set(f'Pitch: {math.degrees(self.pose_data.pitch):+.1f}°')
            self.yaw_var.set(f'Yaw:   {math.degrees(self.pose_data.yaw):+.1f}°')
            
            self.root.after(50, self.update_display)
        
        def run(self):
            self.root.mainloop()


def ros_spin_thread(node):
    """ROS2 spin을 별도 스레드에서 실행"""
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def main():
    rclpy.init()
    
    pose_data = PoseData()
    ros_node = ROSNode(pose_data)
    
    # ROS2를 별도 스레드에서 실행
    ros_thread = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    ros_thread.start()
    
    print("Starting Gripper Pose GUI...")
    print("Move the joints in RViz to see real-time coordinates.")
    
    if USE_PYQT:
        app = QApplication(sys.argv)
        gui = PyQtGUI(pose_data)
        gui.show()
        sys.exit(app.exec_())
    else:
        gui = TkinterGUI(pose_data)
        gui.run()
    
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
