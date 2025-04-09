#!/usr/bin/env python3
# test_dry_run.py

import time
import numpy as np
import math
import signal
import csv
import os
from datetime import datetime
from const import parameter
from robot import Robot
from sensor_interface import SensorInterface
from power_sensor_interface import PowerSensorInterface
from actuator_interface import ActuatorInterface
from controller import Controller
import calculate_degree as calculator

# --- テストモードの設定 ---
# 実際にスラスタを動作させないフラグ
DRY_RUN = True

# --- ログ保存ディレクトリの作成 ---
if not os.path.exists('./csv'):
    os.makedirs('./csv')

# --- 初期設定 ---
print("[INFO] 初期化中...")
const = parameter()
waypoint_list = np.genfromtxt(const.way_point_file, delimiter=',', dtype='float', encoding='utf-8')
robot = Robot(waypoint_list)

try:
    print("[INFO] センサーインターフェース初期化...")
    sensor = SensorInterface()
    print("[INFO] 電力センサーインターフェース初期化...")
    power_sensor = PowerSensorInterface()
    print("[INFO] アクチュエータインターフェース初期化...")
    if DRY_RUN:
        # テストモード用にモックのアクチュエータインターフェースを作成
        class MockActuatorInterface:
            def __init__(self):
                self.closed_pwm = [350] * 8
                self.open_pwm = [500] * 8
                print("[INFO] モックアクチュエータを初期化しました")
            
            def set_keep_mode(self):
                print("[MOCK] KEEPモードに設定")
                
            def set_straight_mode(self):
                print("[MOCK] STRAIGHTモードに設定")
                
            def control_thruster(self, direction, pwm):
                print(f"[MOCK] スラスタ制御: 方向={direction}, PWM={pwm}")
                
            def stop_thruster(self):
                print("[MOCK] スラスタ停止")
                
        actuator = MockActuatorInterface()
    else:
        actuator = ActuatorInterface(pwm_config_file="servo_pwm_config.csv")
    
    print("[INFO] コントローラ初期化...")
    controller = Controller(pwm_strength=50, distance_tolerance=const.way_point_tolerance)
except Exception as e:
    print(f"[ERROR] 初期化中にエラーが発生しました: {e}")
    exit(1)

# --- モード選択 ---
print("[INFO] モード設定...")
mode = const.control_mode  # 0: keep, 1: straight

if mode == 0:
    print("[INFO] KEEPモードを選択")
    actuator.set_keep_mode()
elif mode == 1:
    print("[INFO] STRAIGHTモードを選択")
    actuator.set_straight_mode()

# --- ログ設定 ---
log_data = []
log_header = ['count', 'latitude', 'longitude', 'yaw', 'cmd', 'pwm',
              'voltage', 'current', 'power', 'energy', 'distance', 'heading_diff', 'waypoint_num']
date = time.strftime("%Y%m%d%H%M%S")

# --- 割り込み用ログ関数 ---
def logging(signum=None, frame=None):
    try:
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
        print(f"[LOG] 記録: 距離={diff_distance:.2f}m, 角度差={diff_heading:.2f}度, 電圧={v:.2f}V")
    except Exception as e:
        print(f"[ERROR] ログ記録中にエラーが発生: {e}")

# --- 定期ログ取得のためのシグナル設定 ---
print("[INFO] 定期ログ取得を設定...")
signal.signal(signal.SIGALRM, logging)
signal.setitimer(signal.ITIMER_REAL, 0.5, const.timer)

# --- メインループ ---
print("[INFO] メイン制御ループを開始します。Ctrl+Cで終了します。")
try:
    while True:
        try:
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

            print(f"[INFO] 距離: {diff_distance:.2f} m, 角度差: {diff_heading:.2f} 度, ウェイポイント: {robot.waypoint_num}")

            # 3. 目標に到達しているか確認
            if diff_distance < const.way_point_tolerance:
                print(f"[INFO] ウェイポイント {robot.waypoint_num} に到達しました。")
                robot.update_waypoint()
                if robot.get_waypoint_num() == -1:
                    print("[INFO] ミッション完了！すべてのウェイポイントに到達しました。")
                    actuator.stop_thruster()
                    break

            # 4. 制御コマンドの決定と送信
            direction, pwm = controller.decide_command(diff_distance, diff_heading, mode)
            robot.cmd = direction
            robot.pwm = pwm
            actuator.control_thruster(direction, pwm)

            time.sleep(0.03)  # 陸上テスト用に待機時間を長く設定
        except Exception as e:
            print(f"[ERROR] メインループでエラーが発生: {e}")
            time.sleep(1)  # エラー発生時は少し待機

except KeyboardInterrupt:
    print("[INFO] キーボード割り込みによる停止。")
    actuator.stop_thruster()

finally:
    # 終了処理・ログ保存
    print("[INFO] 終了処理を実行中...")
    actuator.stop_thruster()
    signal.setitimer(signal.ITIMER_REAL, 0)
    
    if log_data:
        log_file = f'./csv/{date}.csv'
        try:
            with open(log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(log_header)
                writer.writerows(log_data)
            print(f"[INFO] ログを保存しました: {log_file}")
        except Exception as e:
            print(f"[ERROR] ログ保存中にエラーが発生: {e}")
    else:
        print("[INFO] ログデータはありません。")
    
    print("[INFO] テスト終了")