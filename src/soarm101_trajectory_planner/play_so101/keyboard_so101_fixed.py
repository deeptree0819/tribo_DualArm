#!/usr/bin/env python3
"""
Keyboard control for SO-Arm 101 robot using LeRobot
P control, keyboard input changes target joint angles
Motor 4 is broken - automatically skipped
"""

import time
import logging
import traceback

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# SO-Arm 101 Joint calibration (adjust as needed)
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # Joint1: zero position offset, scale factor
    ['shoulder_lift', 2.0, 0.97],     # Joint2: zero position offset, scale factor
    ['elbow_flex', 0.0, 1.05],        # Joint3: zero position offset, scale factor
    ['wrist_flex', 0.0, 0.94],        # Joint4: zero position offset, scale factor
    ['wrist_roll', 0.0, 0.5],        # Joint5: zero position offset, scale factor
    ['gripper', 0.0, 1.0], 
]

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
                calibrated_value = apply_joint_calibration(motor_name, value)
                current_positions[motor_name] = calibrated_value
        
        robot_action = {}
        for joint_name, target_pos in zero_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                control_output = kp * error
                new_position = current_pos + control_output
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
                robot_action[f"{joint_name}.pos"] = new_position
        
        if robot_action:
            robot.send_action(robot_action)
        
        if total_error < 2.0:
            print("✓ Returned to start position")
            break
        
        time.sleep(control_period)


def p_control_loop(robot, keyboard, target_positions, start_positions, kp=0.5, control_freq=50):
    """P control loop"""
    control_period = 1.0 / control_freq
    
    print(f"P control loop started (Kp: {kp}, Freq: {control_freq}Hz)")
    
    while True:
        try:
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                for key, value in keyboard_action.items():
                    if key == 'x':
                        print("\nExit command detected")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
                        return
                    
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
                            # 범위 제한: -100 ~ 100 (정규화된 범위)
                            new_target = max(-100, min(100, new_target))
                            target_positions[joint_name] = new_target
                            print(f"  {joint_name}: {current_target:6.1f} → {new_target:6.1f}")
            
            # Get current state
            current_obs = robot.get_observation()
            current_positions = {}
            
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    current_positions[motor_name] = value  # ← 정규화된 값 그대로 사용
            
            # P control (정규화된 범위에서)
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos - current_pos
                    control_output = kp * error
                    new_position = current_pos + control_output
                    # 범위 제한
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
    print("SO-Arm 101 Keyboard Control (LeRobot)")
    print("="*60)
    
    try:
        # ✅ Import SO101Follower (NOT SO100!)
        from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
        from lerobot.robots.so101_follower.so101_follower import SO101Follower
        from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
        
        print("✓ Imports successful (SO101Follower)")
        
        # Get USB port
        port = input("\nEnter SO-Arm 101 USB port (default /dev/ttyACM0): ").strip()
        if not port:
            port = "/dev/ttyACM0"
        print(f"Using port: {port}")
        
        # Configure and connect robot
        print("\nConnecting to robot...")
        robot_config = SO101FollowerConfig(port=port)
        
        
        robot = SO101Follower(robot_config)
        
        # Configure and connect keyboard
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
        print("- Q/A: Joint1 (shoulder_pan) decrease/increase")
        print("- W/S: Joint2 (shoulder_lift) decrease/increase")
        print("- E/D: Joint3 (elbow_flex) decrease/increase")
        print("- R/F: Joint4 (wrist_flex) decrease/increase")
        print("- T/G: Joint5 (wrist_roll) decrease/increase")
        print("- Y/H: Joint6 (gripper) decrease/increase")
        print("- X: Exit program (first return to start position)")
        print("="*60 + "\n")
        
        # Start control
        p_control_loop(robot, keyboard, target_positions, start_positions, kp=0.5, control_freq=50)
        
        # Disconnect
        print("\nDisconnecting...")
        robot.disconnect()
        keyboard.disconnect()
        print("✓ Program ended successfully")
        
    except RuntimeError as e:
        error_msg = str(e)
        if "Missing motor IDs" in error_msg and "4" in error_msg:
            print(f"\n⚠️  Motor 4 check failed (expected - motor is broken)")
            print("Retrying without Motor 4 check...")
            
            # Try again with Motor 4 completely skipped
            try:
                from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
                robot_config = SO101FollowerConfig(port=port)
                robot_config.motors_config.motors.pop(4, None)  # Remove Motor 4 from config
                robot = SO101Follower(robot_config)
                robot.connect()
                print("✓ Connected without Motor 4")
            except Exception as e2:
                print(f"✗ Still failed: {e2}")
        else:
            print(f"\n✗ Error: {e}")
            traceback.print_exc()
            
    except Exception as e:
        print(f"\n✗ Error: {e}")
        traceback.print_exc()
        print("\nTroubleshooting:")
        print("1. Check USB port: ls /dev/ttyACM* /dev/ttyUSB*")
        print("2. Check robot connection and power")
        print("3. Verify LeRobot installation")
        print("4. Check Motor 4 is actually broken/disconnected")
        print("5. sudo chmod 666 /dev/ttyACM0 (if permission denied)")

if __name__ == "__main__":
    main()
