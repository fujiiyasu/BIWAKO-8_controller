# main.py

import time
import numpy as np
import math
import signal
import csv
from const import parameter
from robot import Robot
from sensor_interface import SensorInterface
from power_sensor_interface import PowerSensorInterface
from actuator_interface import ActuatorInterface
from controller import Controller
import calculate_degree as calculator

# --- 初期設定 ---
const = parameter()
waypoint_list = np.genfromtxt(const.way_point_file, delimiter=',', dtype='float', encoding='utf-8')
robot = Robot(waypoint_list)
sensor = SensorInterface()
power_sensor = PowerSensorInterface()
actuator = ActuatorInterface(pwm_config_file="servo_pwm_config.csv")
controller = Controller(pwm_strength=50, distance_tolerance=const.way_point_tolerance)

# --- モード選択 ---
mode = const.control_mode  # 0: keep, 1: straight

if mode == 0:
    actuator.set_keep_mode()
elif mode == 1:
    actuator.set_straight_mode()

# --- ログ設定 ---
log_data = []
log_header = ['count', 'latitude', 'longitude', 'yaw', 'cmd', 'pwm',
              'voltage', 'current', 'power', 'energy', 'distance', 'heading_diff', 'waypoint_num']
date = time.strftime("%Y%m%d%H%M%S")

# --- 割り込み用ログ関数 ---
def logging(signum=None, frame=None):
    # === 1. センサ状態の取得 ===
    lat, lon, yaw = sensor.get_sensor_data()
    robot.update_position(lat, lon)
    robot.update_yaw(yaw)

    # === 2. 電力センサの取得 ===
    power_sensor.update()
    v, c, p, e = power_sensor.get_all()
    robot.update_voltage(v)
    robot.update_current(c)
    robot.update_power(p)

    # === 3. 距離と角度差の再計算（ログ精度向上のため） ===
    current = np.array([lat, lon])
    goal = robot.get_current_waypoint()
    diff_distance = calculator.huveny_distance(goal, current) * 1000  # m
    target_bearing = math.radians(calculator.calculate_bearing(goal, current))
    diff_heading = math.degrees(calculator.limit_angle(target_bearing - yaw))

    robot.diff_distance = diff_distance
    robot.diff_heading = diff_heading

    # === 4. ログ記録 ===
    robot.count_up()
    data = [
        robot.count,
        robot.lat,
        robot.lon,
        math.degrees(robot.yaw),
        robot.cmd,
        robot.pwm,
        v, c, p, e,
        robot.diff_distance,
        robot.diff_heading,
        robot.waypoint_num
    ]
    log_data.append(data)


signal.signal(signal.SIGALRM, logging)
signal.setitimer(signal.ITIMER_REAL, 0.5, const.timer)

# --- メインループ ---
try:
    print("Start main control loop.")
    while True:
        # 1. センサ更新
        lat, lon = robot.get_position()
        yaw = robot.get_yaw()

        # 2. 差分計算
        current = np.array([lat, lon])
        goal = robot.get_current_waypoint()
        diff_distance = calculator.huveny_distance(goal, current) * 1000  # m
        target_bearing = math.radians(calculator.calculate_bearing(goal, current))
        diff_heading = math.degrees(calculator.limit_angle(target_bearing - yaw))

        robot.diff_distance = diff_distance
        robot.diff_heading = diff_heading

        print(f"[INFO] Distance: {diff_distance:.2f} m, Heading: {diff_heading:.2f} deg")

        # 3. 目標に到達しているか確認
        if diff_distance < const.way_point_tolerance:
            print("[INFO] Reached waypoint.")
            robot.update_waypoint()
            if robot.get_waypoint_num() == -1:
                print("[INFO] Mission complete.")
                actuator.stop_thruster()
                break

        # 4. 制御コマンドの決定と送信
        direction, pwm = controller.decide_command(diff_distance, diff_heading, mode)
        robot.cmd = direction
        robot.pwm = pwm
        actuator.control_thruster(direction, pwm)

        time.sleep(0.03)

except KeyboardInterrupt:
    print("[INFO] Keyboard Interrupt. Stopping robot.")
    actuator.stop_thruster()

finally:
    # 終了処理・ログ保存
    actuator.stop_thruster()
    signal.setitimer(signal.ITIMER_REAL, 0)
    with open(f'./csv/{date}.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(log_header)
        writer.writerows(log_data)
    print(f"[INFO] Log saved to ./csv/{date}.csv")
