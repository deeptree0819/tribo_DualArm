#!/usr/bin/env python3
"""
SO-Arm 101 Control with IK Solver
Real-time motor state reading for accurate IK solving
"""

import time
import logging
import traceback
import numpy as np

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

from ikpy_non_ros_ver import ArmControllerNode


# SO-Arm 101 Joint calibration
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],
    ['shoulder_lift', 2.0, 0.97],
    ['elbow_flex', 0.0, 1.05],
    ['wrist_flex', 0.0, 0.94],
    ['wrist_roll', 0.0, 0.5],
    ['gripper', 0.0, 1.0],
]

# Joint name mapping
IK_TO_MOTOR = {
    1: 'shoulder_pan',
    2: 'shoulder_lift',
    3: 'elbow_flex',
    4: 'wrist_flex', # 방향 고정하고 싶으면 사용, 자유롭게 계산 하고 싶으면 주석
    5: 'wrist_roll',
}


def apply_joint_calibration(joint_name, raw_position):
    """Apply joint calibration coefficients"""
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]
            scale = joint_cal[2]
            calibrated_position = (raw_position - offset) * scale
            return calibrated_position
    return raw_position


def move_to_zero_position(robot, duration=3.0, kp=0.5):
    """Move robot to zero position using P control"""
    print("Moving to zero position...")
    
    zero_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }
    
    control_freq = 50
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq
    
    print(f"Duration: {duration}s, Frequency: {control_freq}Hz, Kp: {kp}")
    
    for step in range(total_steps):
        current_obs = robot.get_observation()
        current_positions = {}
        
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value
        
        robot_action = {}
        for joint_name, target_pos in zero_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                control_output = kp * error
                new_position = current_pos + control_output
                new_position = max(-100, min(100, new_position))
                robot_action[f"{joint_name}.pos"] = new_position
        
        if robot_action:
            robot.send_action(robot_action)
        
        if step % 10 == 0:
            progress = (step / total_steps) * 100
            print(f"  Progress: {progress:.1f}%")
        
        time.sleep(step_time)
    
    print("✓ Zero position reached")


def return_to_start_position(robot, start_positions, kp=0.5, control_freq=50):
    """Return to start position using P control"""
    print("Returning to start position...")
    
    control_period = 1.0 / control_freq
    max_steps = int(5.0 * control_freq)
    
    for step in range(max_steps):
        current_obs = robot.get_observation()
        current_positions = {}
        
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value
        
        robot_action = {}
        total_error = 0
        
        for joint_name, target_pos in start_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                total_error += abs(error)
                control_output = kp * error
                new_position = current_pos + control_output
                new_position = max(-100, min(100, new_position))
                robot_action[f"{joint_name}.pos"] = new_position
        
        if robot_action:
            robot.send_action(robot_action)
        
        if total_error < 2.0:
            print("✓ Returned to start position")
            break
        
        time.sleep(control_period)


def get_current_joint_angles(robot):
    """
    🔴 CRITICAL: Read ACTUAL robot state from robot.get_observation()
    NOT from target_positions dictionary or any cached values
    This ensures wrist_flex changes are immediately reflected
    """
    current_obs = robot.get_observation()
    joint_angles = np.zeros(7)
    
    print(f"\n🔍 Reading ACTUAL robot state from robot.get_observation():")
    for key, value in current_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            # Convert to radians - use ACTUAL robot position, not cached
            angle_rad = np.radians(value)
            
            # Map motor to IK joint index
            for ik_idx, motor_map in IK_TO_MOTOR.items():
                if motor_map == motor_name:
                    joint_angles[ik_idx] = angle_rad
                    print(f"   {motor_name}: {value:7.1f}° → {angle_rad:.4f} rad")
                    break
    
    return joint_angles


def apply_ik_solution_to_robot(robot, ik_solver, target_pos, current_joints, kp=0.5, control_freq=50):
    """
    Solve IK and apply to robot using P control
    current_joints = ACTUAL current state from robot
    """
    print(f"\n🎯 IK Target: x={target_pos[0]:.3f}, y={target_pos[1]:.3f}, z={target_pos[2]:.3f}")
    print(f"   Current wrist_flex: {np.degrees(current_joints[4]):.1f}° (will be preserved)")
    
    try:
        # Set current state to IK solver
        ik_solver.current_ik_joints = current_joints.copy()
        
        # Solve IK
        ik_solver.move_to_xyz(target_pos[0], target_pos[1], target_pos[2])
        
        if not ik_solver.is_moving:
            print("❌ IK solution failed")
            return False
        
        # Execute motion with P control
        control_period = 1.0 / control_freq
        max_steps = int(ik_solver.move_duration / control_period + 10)
        
        for step in range(max_steps):
            # Update IK trajectory
            ik_solver.step(dt=control_period)
            
            # Get target positions from IK solver
            ik_state = ik_solver.get_joint_state()
            
            target_positions = {
                'shoulder_pan': np.degrees(ik_state['pan']),
                'shoulder_lift': np.degrees(ik_state['lift']),
                'elbow_flex': np.degrees(ik_state['elbow']),
                'wrist_flex': np.degrees(ik_state['flex']),
                'wrist_roll': np.degrees(ik_state['roll']),
                'gripper': np.degrees(ik_state['gripper']),
            }
            
            # Get ACTUAL current robot state
            current_obs = robot.get_observation()
            current_positions = {}
            
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    current_positions[motor_name] = value
            
            # P control
            robot_action = {}
            total_error = 0
            
            for joint_name, target_pos_deg in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos_deg - current_pos
                    total_error += abs(error)
                    control_output = kp * error
                    new_position = current_pos + control_output
                    new_position = max(-100, min(100, new_position))
                    robot_action[f"{joint_name}.pos"] = new_position
            
            if robot_action:
                robot.send_action(robot_action)
            
            if step % 5 == 0:
                print(f"  Moving... error={total_error:.2f}°")
            
            if not ik_solver.is_moving and total_error < 5.0:
                print("✓ Position reached")
                return True
            
            time.sleep(control_period)
        
        print("✓ Motion completed")
        return True
        
    except Exception as e:
        print(f"❌ IK execution failed: {e}")
        traceback.print_exc()
        return False


def interactive_ik_mode(robot, ik_solver, target_positions, kp=0.5, control_freq=50):
    """
    Interactive IK mode with real-time motor feedback
    """
    print("\n" + "="*60)
    print("IK Interactive Mode")
    print("-"*60)
    print("Enter target coordinates (x, y, z in meters)")
    print("Example: 0.3 0.0 0.2")
    print("\nOr adjust wrist_flex with manual keys:")
    print("  R/F: wrist_flex -/+ (applied immediately)")
    print("\nCommands:")
    print("  exit: Return to keyboard control")
    print("  x: Exit program")
    print("="*60 + "\n")
    
    while True:
        try:
            user_input = input("Target XYZ or key: ").strip().lower()
            
            if user_input == 'exit':
                print("Exiting IK mode → Back to keyboard control...")
                return 'keyboard'
            
            # Single character keyboard command
            if len(user_input) == 1:
                joint_controls = {
                    'q': ('shoulder_pan', -5),
                    'a': ('shoulder_pan', 5),
                    'w': ('shoulder_lift', -5),
                    's': ('shoulder_lift', 5),
                    'e': ('elbow_flex', -5),
                    'd': ('elbow_flex', 5),
                    'r': ('wrist_flex', -5),
                    'f': ('wrist_flex', 5),
                    't': ('wrist_roll', -5),
                    'g': ('wrist_roll', 5),
                    'y': ('gripper', -5),
                    'h': ('gripper', 5),
                }
                
                if user_input == 'x':
                    print("Exit command (x) → Program termination...")
                    return 'exit'
                
                if user_input in joint_controls:
                    joint_name, delta = joint_controls[user_input]
                    if joint_name in target_positions:
                        current_target = target_positions[joint_name]
                        new_target = current_target + delta
                        new_target = max(-100, min(100, new_target))
                        target_positions[joint_name] = new_target
                        
                        # Apply immediately to robot
                        current_obs = robot.get_observation()
                        if f"{joint_name}.pos" in current_obs:
                            current_pos = current_obs[f"{joint_name}.pos"]
                            error = new_target - current_pos
                            control_output = kp * error
                            final_pos = current_pos + control_output
                            final_pos = max(-100, min(100, final_pos))
                            robot.send_action({f"{joint_name}.pos": final_pos})
                        
                        print(f"  ✅ {joint_name}: {current_target:6.1f} → {new_target:6.1f}")
                    continue
            
            # Parse as coordinates
            coords = user_input.split()
            if len(coords) == 3:
                try:
                    x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                    
                    # 🔴 CRITICAL: Get ACTUAL current robot state
                    current_joints = get_current_joint_angles(robot)
                    
                    # Solve IK with actual current state
                    # apply_ik_solution_to_robot(robot, ik_solver, [x, y, z], current_joints, kp, control_freq)

                    success = apply_ik_solution_to_robot(robot, ik_solver, [x, y, z], current_joints, kp, control_freq)
        
                    # ✅ IK 실패했을 때 근처 좌표 자동 추천
                    if not success:
                        print("❌ Direct IK failed")
                        
                        # 근처 좌표 자동 추천
                        recommended_pos = ik_solver.find_nearby_reachable_position(x, y, z, search_radius=0.05)
                        
                        if recommended_pos is not None:
                            print(f"\n💡 Would you like to go to recommended position? (y/n)")
                            user_choice = input().strip().lower()
                            if user_choice == 'y':
                                apply_ik_solution_to_robot(robot, ik_solver, recommended_pos, current_joints, kp, control_freq)
                    
                except ValueError:
                    print(f"   coords={coords}")
            else:
                print("❌ Invalid input")
            
        except KeyboardInterrupt:
            print("\nExiting IK mode...")
            return 'exit'
        except Exception as e:
            print(f"Error: {e}")
            traceback.print_exc()
            return 'exit'


def p_control_loop(robot, keyboard, ik_solver, target_positions, start_positions, kp=0.5, control_freq=50):
    """Main P control loop"""
    control_period = 1.0 / control_freq
    
    print(f"P control loop started (Kp: {kp}, Freq: {control_freq}Hz)")
    print("Press 'i' for IK coordinate mode\n")
    
    while True:
        try:
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                for key, value in keyboard_action.items():
                    if key == 'x':
                        print("\nExit command detected")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
                        return
                    
                    if key == 'i':
                        print("\n⚙️  Entering IK mode...")

                        mode_result = interactive_ik_mode(robot, ik_solver, target_positions, kp, control_freq)

                        if mode_result == 'exit':
                            print("\n⚠️  Exit command detected in IK mode")
                            return_to_start_position(robot, start_positions, 0.2, control_freq)
                            return
                        else:
                            print("Back to keyboard control...\n")
                        continue
                    
                    # Manual joint controls
                    joint_controls = {
                        'q': ('shoulder_pan', -5),
                        'a': ('shoulder_pan', 5),
                        'w': ('shoulder_lift', -5),
                        's': ('shoulder_lift', 5),
                        'e': ('elbow_flex', -5),
                        'd': ('elbow_flex', 5),
                        'r': ('wrist_flex', -5),
                        'f': ('wrist_flex', 5),
                        't': ('wrist_roll', -5),
                        'g': ('wrist_roll', 5),
                        'y': ('gripper', -5),
                        'h': ('gripper', 5),
                    }
                    
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name in target_positions:
                            current_target = target_positions[joint_name]
                            new_target = current_target + delta
                            new_target = max(-100, min(100, new_target))
                            target_positions[joint_name] = new_target
                            print(f"  {joint_name}: {current_target:6.1f} → {new_target:6.1f}")
            
            # Get ACTUAL current state from robot
            current_obs = robot.get_observation()
            current_positions = {}
            
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    current_positions[motor_name] = value
            
            # P control
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos - current_pos
                    control_output = kp * error
                    new_position = current_pos + control_output
                    new_position = max(-100, min(100, new_position))
                    robot_action[f"{joint_name}.pos"] = new_position
            
            if robot_action:
                robot.send_action(robot_action)
            
            time.sleep(control_period)
            
        except KeyboardInterrupt:
            print("\nUser interrupted")
            break
        except Exception as e:
            print(f"Error: {e}")
            traceback.print_exc()
            break


def main():
    """Main function"""
    print("="*60)
    print("SO-Arm 101 Control with IK Solver")
    print("Motor 4 replaced with Motor 6")
    print("Real-time actual motor state reading")
    print("="*60)
    
    # Initialize IK Solver
    print("\n[1/3] Initializing IK Solver...")
    try:
        urdf_path = input("Enter URDF path: ").strip()
        if not urdf_path:
            urdf_path = "/home/deeptree/dev_ws/ros2lerobot_ws/src/dt_arm_description/urdf/soarm101_40mmUP.urdf"
        
        ik_solver = ArmControllerNode(urdf_path)
        print("✓ IK Solver initialized")
    except Exception as e:
        print(f"✗ IK Solver initialization failed: {e}")
        ik_solver = None
    
    try:
        # Import robot and keyboard
        print("\n[2/3] Importing LeRobot modules...")
        from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
        from lerobot.robots.so101_follower.so101_follower import SO101Follower
        from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
        
        print("✓ Imports successful")
        
        # Get USB port
        port = input("\nEnter SO-Arm 101 USB port (default /dev/ttyACM0): ").strip()
        if not port:
            port = "/dev/ttyACM0"
        print(f"Using port: {port}")
        
        # Configure and connect robot
        print("\n[3/3] Connecting to robot...")
        robot_config = SO101FollowerConfig(port=port)
        
        robot = SO101Follower(robot_config)
        keyboard_config = KeyboardTeleopConfig()
        keyboard = KeyboardTeleop(keyboard_config)
        
        robot.connect()
        keyboard.connect()
        print("✓ Connected successfully!")
        
        # Calibration
        print()
        while True:
            calibrate_choice = input("Calibrate robot? (y/n): ").strip().lower()
            if calibrate_choice in ['y', 'yes']:
                print("Calibrating...")
                robot.calibrate()
                print("✓ Calibration done")
                break
            elif calibrate_choice in ['n', 'no']:
                print("Using previous calibration")
                break
        
        # Read start positions
        print("\nReading start positions...")
        start_obs = robot.get_observation()
        start_positions = {}
        
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value)
        
        print("Start positions:")
        for joint_name, position in sorted(start_positions.items()):
            print(f"  {joint_name:12s}: {position:6.1f}°")
        
        # Move to zero
        print()
        move_to_zero_position(robot, duration=3.0, kp=0.5)
        
        # Initialize targets
        target_positions = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        
        # Print instructions
        print("\n" + "="*60)
        print("Keyboard Control Instructions:")
        print("-"*60)
        print("Manual Joint Control (Q/A/W/S/E/D/R/F/T/G/Y/H)")
        print("IK Mode: I")
        print("Exit: X")
        print("="*60 + "\n")
        
        # Start control
        if ik_solver:
            p_control_loop(robot, keyboard, ik_solver, target_positions, start_positions, kp=0.5, control_freq=50)
        else:
            print("⚠️  No IK solver")
        
        # Disconnect
        print("\nDisconnecting...")
        robot.disconnect()
        keyboard.disconnect()
        print("✓ Program ended successfully")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        traceback.print_exc()


if __name__ == "__main__":
    main()
