#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
BIWAKO-8 Thruster Test
----------------------
This script tests the thruster rotation direction and strength of BIWAKO-8 robot.
It uses the ActuatorInterface to control the thrusters in KEEP mode.
"""

import time
import os
import sys
import signal

# Add parent directory to path to import modules
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# Import the existing ActuatorInterface class
from actuator_interface import ActuatorInterface

# Definition of available directions in KEEP mode
# These match the Arduino program's direction definitions
DIRECTION_STOP = 0    # Stop all thrusters
DIRECTION_FORWARD = 1 # Forward - T1,T2 positive, T3,T4 negative
DIRECTION_BACKWARD = 2 # Backward - T1,T2 negative, T3,T4 positive
DIRECTION_RIGHT = 3   # Right - T1,T3 positive, T2,T4 negative
DIRECTION_LEFT = 4    # Left - T1,T3 negative, T2,T4 positive
DIRECTION_ROTATE_CW = 7  # Clockwise rotation
DIRECTION_ROTATE_CCW = 8  # Counter-clockwise rotation

def print_help():
    """Print help message with available commands"""
    print("\n" + "="*50)
    print("BIWAKO-8 Thruster Test (KEEP MODE)")
    print("="*50)
    print("Direction commands:")
    print("  0 - Stop all thrusters")
    print("  1 - Forward")
    print("  2 - Backward")
    print("  3 - Right")
    print("  4 - Left")
    print("  7 - Rotate clockwise")
    print("  8 - Rotate counter-clockwise")
    print("\nStrength commands (after direction):")
    print("  0-100 - Thruster strength percentage")
    print("\nOther commands:")
    print("  h - Show this help message")
    print("  q - Quit")
    print("="*50)
    print("\nExample: '1 50' sets forward direction at 50% strength")

def keyboard_control(actuator):
    """Interactive keyboard control loop"""
    print_help()
    
    # Ensure the robot is in KEEP mode for proper thruster control
    print("\n[INFO] Setting robot to KEEP mode for testing")
    actuator.set_keep_mode()
    time.sleep(1.0)
    
    while True:
        try:
            cmd = input("\nCommand [direction strength/h/q]: ").strip().lower()
            
            if cmd == 'h':
                print_help()
                continue
            elif cmd == 'q':
                actuator.stop_thruster()
                print("[INFO] Thrusters stopped. Exiting...")
                break
                
            # Parse command for direction and strength
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    direction = int(parts[0])
                    pwm_strength = int(parts[1])
                    
                    # Validate direction
                    if direction not in [0, 1, 2, 3, 4, 7, 8]:
                        print(f"[ERROR] Invalid direction: {direction}")
                        print("Valid directions: 0 (stop), 1 (forward), 2 (backward), 3 (right), 4 (left), 7 (CW), 8 (CCW)")
                        continue
                        
                    # Validate strength
                    if pwm_strength < 0 or pwm_strength > 100:
                        print(f"[ERROR] Invalid strength: {pwm_strength}")
                        print("Strength must be between 0 and 100")
                        continue
                    
                    # Send command to actuator
                    print(f"[INFO] Direction: {direction}, Strength: {pwm_strength}%")
                    actuator.control_thruster(direction, pwm_strength)
                    
                except ValueError:
                    print("[ERROR] Invalid input format. Use 'direction strength' (e.g., '1 50')")
            else:
                print("[ERROR] Invalid input format. Use 'direction strength' (e.g., '1 50')")
        
        except KeyboardInterrupt:
            actuator.stop_thruster()
            print("\n[INFO] Thrusters stopped. Exiting...")
            break
        except Exception as e:
            print(f"[ERROR] {e}")

def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    print("\n[INFO] Exiting...")
    sys.exit(0)

def main():
    """Main function"""
    # Set up signal handler for graceful exit
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Initialize actuator interface
        actuator = ActuatorInterface()
        
        # Start keyboard control loop
        keyboard_control(actuator)
        
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        # Clean shutdown
        try:
            print("[INFO] Stopping thrusters...")
            actuator.stop_thruster()
        except:
            pass
        print("[INFO] Exiting...")

if __name__ == "__main__":
    main()