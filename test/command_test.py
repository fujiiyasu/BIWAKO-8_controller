import math
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle
import matplotlib.transforms as mtransforms

# Import modules from parent directory
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import modules from parent directory
import calculate_degree as calculator
from controller import Controller

def test_bearing_calculation():
    # Test case with fixed coordinates
    current_lat = 34.806953
    current_lon = 135.561388
    target_lat = 34.806918
    target_lon = 135.561436
    yaw_deg = float(input("Robot's current heading angle (degrees, 0-360): "))
    """
    current_lat = float(input("Current latitude (e.g., 35.0127): "))
    current_lon = float(input("Current longitude (e.g., 135.7654): "))
    target_lat = float(input("Target latitude (e.g., 35.0130): "))
    target_lon = float(input("Target longitude (e.g., 135.7658): "))
    yaw_deg = float(input("Robot's current heading angle (degrees, 0-360): "))
    """

    # Fixed mode selection (0: keep, 1: straight)
    # mode = int(input("Movement mode (0: keep, 1: straight): "))
    mode = 0
    
    # Current and target positions
    current_point = np.array([current_lat, current_lon])
    target_point = np.array([target_lat, target_lon])
    
    # Distance calculation
    diff_distance = calculator.huveny_distance(target_point, current_point) * 1000  # m
    
    # Bearing calculation
    target_bearing_deg = calculator.calculate_bearing(target_point, current_point)
    target_bearing = math.radians(target_bearing_deg)
    
    # Calculate heading difference
    yaw = math.radians(yaw_deg)
    diff_heading = math.degrees(calculator.limit_angle(target_bearing - yaw))
    
    # Calculate control command
    controller = Controller(pwm_strength=50, distance_tolerance=2.0)
    direction, pwm = controller.decide_command(diff_distance, diff_heading, mode)
    
    # Convert direction to text
    direction_text = direction_to_text(direction, mode)
    
    # Display results
    print("\n===== Calculation Results =====")
    print(f"Distance: {diff_distance:.2f} m")
    print(f"Target bearing: {target_bearing_deg:.2f}°")
    print(f"Heading difference: {diff_heading:.2f}°")
    print(f"Movement direction: {heading_to_direction_indicator(diff_heading, mode)}")
    print(f"Command: {direction} ({direction_text}), PWM: {pwm}")
    
    # Display graphical representation
    plot_navigation_info(current_lat, current_lon, target_lat, target_lon, 
                        yaw_deg, target_bearing_deg, diff_heading, direction, mode)
    
    return diff_distance, target_bearing_deg, diff_heading, direction, pwm

def direction_to_text(direction, mode):
    """Function to convert direction code to text"""
    # Mode 0: keep (4-direction movement)
    if mode == 0:
        direction_map = {
            0: "Stop",
            1: "Forward",
            2: "Backward",
            3: "Right",
            4: "Left"
        }
    # Mode 1: straight (8-direction movement)
    elif mode == 1:
        direction_map = {
            0: "Stop",
            1: "Forward",
            2: "Backward",
            3: "Forward-Right",
            4: "Backward-Right",
            5: "Backward-Left",
            6: "Forward-Left"
        }
    else:
        return "Unknown"
    
    return direction_map.get(direction, f"Undefined direction:{direction}")

def heading_to_direction_indicator(diff_heading, mode=1):
    """Function to indicate direction with arrow based on heading difference"""
    # Keep mode: 4-direction display
    if mode == 0:
        if -45.0 <= diff_heading < 45.0:
            return "↑ (Forward)"
        elif 45.0 <= diff_heading < 135.0:
            return "→ (Right)"
        elif -135.0 <= diff_heading < -45.0:
            return "← (Left)"
        elif diff_heading >= 135.0 or diff_heading < -135.0:
            return "↓ (Backward)"
        else:
            return "? (Unknown)"
    
    # Straight mode: 8-direction display
    else:
        if -22.5 <= diff_heading < 22.5:
            return "↑ (Forward)"
        elif 22.5 <= diff_heading < 67.5:
            return "↗ (Forward-Right)"
        elif 67.5 <= diff_heading < 112.5:
            return "→ (Right)"
        elif 112.5 <= diff_heading < 157.5:
            return "↘ (Backward-Right)"
        elif diff_heading >= 157.5 or diff_heading < -157.5:
            return "↓ (Backward)"
        elif -157.5 <= diff_heading < -112.5:
            return "↙ (Backward-Left)"
        elif -112.5 <= diff_heading < -67.5:
            return "← (Left)"
        elif -67.5 <= diff_heading < -22.5:
            return "↖ (Forward-Left)"
        else:
            return "? (Unknown)"

def plot_navigation_info(current_lat, current_lon, target_lat, target_lon, 
                         yaw_deg, target_bearing_deg, diff_heading, command, mode):
    """
    Function to graphically display navigation information
    """
    plt.figure(figsize=(10, 10))
    
    # Scale factors for latitude and longitude (for visualization)
    lat_scale = 111000  # meters per degree of latitude
    lon_scale = 111000 * math.cos(math.radians(current_lat))  # meters per degree of longitude
    
    # Calculate distance between current and target positions in meters
    dx = (target_lon - current_lon) * lon_scale
    dy = (target_lat - current_lat) * lat_scale
    distance = math.sqrt(dx**2 + dy**2)
    
    # Set plot range (in meters)
    max_range = max(100, distance * 1.5)  # At least 100m, or 1.5 times the distance
    
    # Convert to relative coordinates with current position as origin (in meters)
    x_current, y_current = 0, 0
    x_target = (target_lon - current_lon) * lon_scale
    y_target = (target_lat - current_lat) * lat_scale
    
    # Set plot limits
    plt.xlim(-max_range/2, max_range/2)
    plt.ylim(-max_range/2, max_range/2)
    
    # Display grid lines
    plt.grid(True)
    
    # Set axis labels
    plt.xlabel('East-West Direction [m]')
    plt.ylabel('North-South Direction [m]')
    plt.title('Navigation Information')
    
    # Display current position as blue circle
    current_point = plt.scatter(x_current, y_current, color='blue', s=100, zorder=3, label='Current Position')
    
    # Display small circle around current position (2m radius for tolerance)
    plt.gca().add_patch(Circle((x_current, y_current), 2, fill=False, color='blue', linestyle='--'))
    
    # Display target position as red star
    target_point = plt.scatter(x_target, y_target, color='red', s=200, marker='*', zorder=3, label='Target Position')
    
    # Display robot heading (yaw) as blue arrow
    arrow_length = max_range / 10
    yaw_rad = math.radians(yaw_deg)
    dx_yaw = arrow_length * math.sin(yaw_rad)
    dy_yaw = arrow_length * math.cos(yaw_rad)
    yaw_arrow = plt.arrow(x_current, y_current, dx_yaw, dy_yaw, 
                         head_width=arrow_length/4, head_length=arrow_length/3, 
                         fc='blue', ec='blue', zorder=2, label='Robot Heading')
    
    # FIXED: Display target bearing directly towards the target point
    # Calculate direction vector from current to target
    if distance > 0:  # Avoid division by zero
        dx_to_target = x_target
        dy_to_target = y_target
        # Normalize to arrow_length
        scale_factor = arrow_length / distance
        dx_target_arrow = dx_to_target * scale_factor
        dy_target_arrow = dy_to_target * scale_factor
    else:
        # If distance is zero, use the calculated bearing (fallback)
        target_bearing_rad = math.radians(target_bearing_deg)
        dx_target_arrow = arrow_length * math.sin(target_bearing_rad)
        dy_target_arrow = arrow_length * math.cos(target_bearing_rad)
    
    target_arrow = plt.arrow(x_current, y_current, dx_target_arrow, dy_target_arrow, 
                            head_width=arrow_length/4, head_length=arrow_length/3, 
                            fc='red', ec='red', zorder=2, label='Target Bearing')
    
    # MODIFIED: Display command direction as green arrow relative to robot heading
    command_direction = get_command_direction(command, mode)
    if command_direction is not None:
        # 現在のロボットの方位を基準とした相対角度を計算
        cmd_angle_rad = math.radians(command_direction)
        
        # ロボットの方位から見た相対角度でコマンド矢印を描画
        # ロボットの方位角を0度として、そこからの相対角度でコマンド方向を計算
        # (先に計算したヨー角（青い矢印）の方向を基準として、そこからの相対角度を計算)
        relative_cmd_angle_rad = yaw_rad + cmd_angle_rad
        
        dx_cmd = arrow_length * math.sin(relative_cmd_angle_rad)
        dy_cmd = arrow_length * math.cos(relative_cmd_angle_rad)
        
        cmd_arrow = plt.arrow(x_current, y_current, dx_cmd, dy_cmd, 
                             head_width=arrow_length/4, head_length=arrow_length/3, 
                             fc='green', ec='green', zorder=2, label='Command Direction')
    
    # Display text information
    info_text = f"Distance: {distance:.2f} m\n" \
                f"Robot Heading: {yaw_deg:.1f}°\n" \
                f"Target Bearing: {target_bearing_deg:.1f}°\n" \
                f"Heading Diff: {diff_heading:.1f}°\n" \
                f"Command: {direction_to_text(command, mode)}"
    
    plt.text(-max_range/2 * 0.9, max_range/2 * 0.9, info_text,
             bbox=dict(facecolor='white', alpha=0.7))
    
    # Display cardinal directions
    plt.text(max_range/2 * 0.9, 0, "E", fontsize=12)
    plt.text(-max_range/2 * 0.9, 0, "W", fontsize=12)
    plt.text(0, max_range/2 * 0.9, "N", fontsize=12)
    plt.text(0, -max_range/2 * 0.9, "S", fontsize=12)
    
    # Display legend
    plt.legend(loc='lower right')
    
    plt.axis('equal')
    plt.show()

def get_command_direction(command, mode):
    """Function to get the direction angle corresponding to a command"""
    # Mode 0: keep (4-direction movement)
    if mode == 0:
        direction_map = {
            0: None,     # Stop
            1: 0,        # Forward (0 degrees)
            2: 180,      # Backward (180 degrees)
            3: 90,       # Right (90 degrees)
            4: 270       # Left (270 degrees)
        }
    # Mode 1: straight (8-direction movement)
    elif mode == 1:
        direction_map = {
            0: None,     # Stop
            1: 0,        # Forward (0 degrees)
            2: 180,      # Backward (180 degrees)
            3: 45,       # Forward-Right (45 degrees)
            4: 135,      # Backward-Right (135 degrees)
            5: 225,      # Backward-Left (225 degrees)
            6: 315       # Forward-Left (315 degrees)
        }
    else:
        return None
    
    return direction_map.get(command)

if __name__ == "__main__":
    try:
        test_bearing_calculation()
    except Exception as e:
        print(f"An error occurred: {e}")
    
    # Run additional test cases if desired
    while input("\nWould you like to try another test case? (y/n): ").lower() == 'y':
        try:
            test_bearing_calculation()
        except Exception as e:
            print(f"An error occurred: {e}")