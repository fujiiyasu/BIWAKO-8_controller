#!/usr/bin/env python3
import sys
import os
import time
import numpy as np
import math
import signal
import csv
import traceback
from datetime import datetime

# カレントディレクトリを変更してインポートパスを設定
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, '..'))
os.chdir(project_root)
sys.path.append(project_root)

from const import parameter
from robot import Robot
from sensor_interface import SensorInterface
from power_sensor_interface import PowerSensorInterface
from actuator_interface import ActuatorInterface
from controller import Controller
import calculate_degree as calculator

# --- 方向を文字列に変換する関数 ---
def direction_to_text(direction, mode):
    # モード0: keep (4方向移動)
    if mode == 0:
        direction_map = {
            0: "停止",
            1: "前進",
            2: "後退",
            3: "右",
            4: "左"
        }
    # モード1: straight (8方向移動)
    elif mode == 1:
        direction_map = {
            0: "停止",
            1: "前進",
            2: "後退",
            3: "前右斜め",
            4: "後右斜め",
            5: "後左斜め",
            6: "前左斜め"
        }
    else:
        return "不明"
    
    return direction_map.get(direction, f"未定義方向:{direction}")

# --- 方位角から進行方向を視覚的に表示する関数 ---
def heading_to_direction_indicator(diff_heading):
    """
    方位角の差から進行方向を↑↓←→などの矢印で表示する
    
    Args:
        diff_heading: 目標方位と現在方位の差（度）-180〜180の範囲
    
    Returns:
        方向を示す矢印の文字列
    """
    # 角度を45度区切りの8方向にマッピング
    if -22.5 <= diff_heading < 22.5:
        return "↑ (前)"
    elif 22.5 <= diff_heading < 67.5:
        return "↗ (右前)"
    elif 67.5 <= diff_heading < 112.5:
        return "→ (右)"
    elif 112.5 <= diff_heading < 157.5:
        return "↘ (右後)"
    elif diff_heading >= 157.5 or diff_heading < -157.5:
        return "↓ (後)"
    elif -157.5 <= diff_heading < -112.5:
        return "↙ (左後)"
    elif -112.5 <= diff_heading < -67.5:
        return "← (左)"
    elif -67.5 <= diff_heading < -22.5:
        return "↖ (左前)"
    else:
        return "? (不明)"

# --- 現在位置と次のウェイポイントの関係を表示する関数 ---
def display_navigation_guide(current_lat, current_lon, target_lat, target_lon, yaw, diff_heading, diff_distance):
    """
    現在地と目標地点の関係を視覚的に表示する
    
    Args:
        current_lat: 現在の緯度
        current_lon: 現在の経度
        target_lat: 目標ウェイポイントの緯度
        target_lon: 目標ウェイポイントの経度
        yaw: 現在の機体方位角（ラジアン）
        diff_heading: 目標方位と現在方位の差（度）
        diff_distance: 目標地点までの距離（m）
    """
    # 現在の方位（度）
    current_heading_deg = math.degrees(yaw)
    
    # 目標方位（度）
    target_heading_deg = (current_heading_deg + diff_heading) % 360
    
    # 方向指示器を取得
    direction_indicator = heading_to_direction_indicator(diff_heading)
    
    # 角度を方位に変換
    def heading_to_compass(deg):
        compass_points = ["北", "北北東", "北東", "東北東", "東", "東南東", "南東", "南南東",
                         "南", "南南西", "南西", "西南西", "西", "西北西", "北西", "北北西"]
        idx = round(deg / 22.5) % 16
        return compass_points[idx]
    
    current_compass = heading_to_compass(current_heading_deg)
    target_compass = heading_to_compass(target_heading_deg)
    
    # 出力
    nav_box = f"""
┌───────────────────────── ナビゲーション情報 ─────────────────────────┐
│ 現在位置: 緯度 {current_lat:.6f}°, 経度 {current_lon:.6f}°   │
│ 目標地点: 緯度 {target_lat:.6f}°, 経度 {target_lon:.6f}°   │
│                                                                      │
│ 現在方位: {current_heading_deg:.1f}° ({current_compass})                     │
│ 目標方位: {target_heading_deg:.1f}° ({target_compass})                     │
│ 方位差  : {diff_heading:.1f}°                                        │
│                                                                      │
│ 進行方向: {direction_indicator}                                            │
│ 残り距離: {diff_distance:.2f} m                                       │
└──────────────────────────────────────────────────────────────────────┘
"""
    print(nav_box)

# --- 安全にセンサーデータを取得する関数 ---
def safe_get_sensor_data(sensor, default_lat=35.0, default_lon=135.0, default_yaw=0.0):
    try:
        # 安全なメソッドを使用 (修正版SensorInterfaceが実装している場合)
        if hasattr(sensor, 'get_sensor_data_safe'):
            lat, lon, yaw = sensor.get_sensor_data_safe()
        else:
            # 互換性のためのフォールバック
            lat, lon, yaw = sensor.get_sensor_data()
        return lat, lon, yaw
    except Exception as e:
        print(f"[WARN] センサーデータ取得中にエラー: {e}")
        print(f"[WARN] デフォルト値を使用します: lat={default_lat}, lon={default_lon}, yaw={default_yaw}")
        return default_lat, default_lon, default_yaw

# --- 安全に電力センサーデータを取得する関数 ---
def safe_get_power_data(power_sensor, default_v=12.0, default_c=0.0, default_p=0.0, default_e=0.0):
    try:
        power_sensor.update()
        v, c, p, e = power_sensor.get_all()
        return v, c, p, e
    except Exception as e:
        print(f"[WARN] 電力センサーデータ取得中にエラー: {e}")
        print(f"[WARN] デフォルト値を使用します: v={default_v}, c={default_c}, p={default_p}, e={default_e}")
        return default_v, default_c, default_p, default_e

# --- テストモードの設定 ---
# 実際にスラスタを動作させないフラグ
DRY_RUN = True

# --- ログ保存ディレクトリの作成 ---
if not os.path.exists('./csv'):
    os.makedirs('./csv')

# --- グローバル変数 ---
sensor = None
power_sensor = None
actuator = None
controller = None
robot = None
const = None
log_data = []
log_header = ['count', 'latitude', 'longitude', 'yaw', 'cmd', 'pwm',
              'voltage', 'current', 'power', 'energy', 'distance', 'heading_diff', 'waypoint_num']
date = time.strftime("%Y%m%d%H%M%S")

# --- 割り込み用ログ関数 ---
def logging(signum=None, frame=None):
    global robot, sensor, power_sensor, log_data
    
    try:
        # === 1. センサ状態の取得 ===
        try:
            lat, lon, yaw = safe_get_sensor_data(sensor, 
                                                default_lat=robot.lat, 
                                                default_lon=robot.lon, 
                                                default_yaw=robot.yaw)
            robot.update_position(lat, lon)
            robot.update_yaw(yaw)
        except Exception as e:
            print(f"[ERROR] センサーデータ更新中にエラー: {e}")
            print(traceback.format_exc())
            # エラーがあっても継続

        # === 2. 電力センサの取得 ===
        try:
            v, c, p, e = safe_get_power_data(power_sensor)
            robot.update_voltage(v)
            robot.update_current(c)
            robot.update_power(p)
        except Exception as e:
            print(f"[ERROR] 電力センサー更新中にエラー: {e}")
            print(traceback.format_exc())
            # エラーがあっても継続

        # === 3. 距離と角度差の再計算（ログ精度向上のため） ===
        try:
            current = np.array([robot.lat, robot.lon])
            goal = robot.get_current_waypoint()
            diff_distance = calculator.huveny_distance(goal, current) * 1000  # m
            target_bearing = math.radians(calculator.calculate_bearing(goal, current))
            diff_heading = math.degrees(calculator.limit_angle(target_bearing - robot.yaw))

            robot.diff_distance = diff_distance
            robot.diff_heading = diff_heading
        except Exception as e:
            print(f"[ERROR] 位置計算中にエラー: {e}")
            print(traceback.format_exc())
            # エラーがあっても継続

        # === 4. ログ記録 ===
        try:
            robot.count_up()
            data = [
                robot.count,
                robot.lat,
                robot.lon,
                math.degrees(robot.yaw),
                robot.cmd,
                robot.pwm,
                robot.voltage, robot.current, robot.power, robot.consumed_energy,
                robot.diff_distance,
                robot.diff_heading,
                robot.waypoint_num
            ]
            log_data.append(data)
            print(f"[LOG] 記録: 距離={robot.diff_distance:.2f}m, 角度差={robot.diff_heading:.2f}度, 電圧={robot.voltage:.2f}V")
            
            # 10回ごとに詳細なセンサー情報を表示（デバッグ用）
            if robot.count % 10 == 0:
                print(f"[DEBUG] センサー生値: lat={robot.lat:.6f}, lon={robot.lon:.6f}, yaw={math.degrees(robot.yaw):.2f}°")
                
        except Exception as e:
            print(f"[ERROR] ログデータ作成中にエラー: {e}")
            print(traceback.format_exc())
            # エラーがあっても継続

    except Exception as e:
        print(f"[ERROR] ロギング処理全体でエラー: {e}")
        print(traceback.format_exc())
        # ロギングでエラーが発生しても処理は継続させる

def main():
    global sensor, power_sensor, actuator, controller, robot, const
    
    # --- 初期設定 ---
    print("[INFO] 初期化中...")
    
    try:
        const = parameter()
        # ウェイポイントの読み込み
        try:
            waypoint_list = np.genfromtxt(const.way_point_file, delimiter=',', dtype='float', encoding='utf-8')
            print(f"[INFO] ウェイポイントを読み込みました: {const.way_point_file}")
        except Exception as e:
            print(f"[ERROR] ウェイポイント読み込み失敗: {e}")
            # 緊急時のデフォルトウェイポイント (近くの点を設定)
            waypoint_list = np.array([[35.0, 135.0], [35.001, 135.001]])
            print("[WARN] デフォルトウェイポイントを使用します")
        
        robot = Robot(waypoint_list)

        print("[INFO] センサーインターフェース初期化...")
        try:
            sensor = SensorInterface()
            # デバッグフラグをオフに設定（必要に応じてオンに）
            if hasattr(sensor, 'debug'):
                sensor.debug = False
        except Exception as e:
            print(f"[ERROR] センサーインターフェース初期化失敗: {e}")
            print("[WARN] モックセンサーを使用します")
            # モックセンサーを作成
            class MockSensorInterface:
                def __init__(self):
                    self.prev_lat = 35.0
                    self.prev_lon = 135.0
                    self.prev_yaw = 0.0
                    self.debug = False
                    print("[INFO] モックセンサーを初期化しました")
                
                def get_sensor_data(self):
                    # 少しずつ移動・回転するシミュレーション
                    self.prev_lat += 0.00001
                    self.prev_lon += 0.00001
                    self.prev_yaw = (self.prev_yaw + 0.01) % (2 * 3.14159)
                    return self.prev_lat, self.prev_lon, self.prev_yaw
                
                def get_sensor_data_safe(self):
                    return self.get_sensor_data()
                    
                def wait_heartbeat(self):
                    pass
                
                def update_attitude(self):
                    self.prev_yaw = (self.prev_yaw + 0.01) % (2 * 3.14159)
                    return self.prev_yaw
                
                def update_attitude_safe(self):
                    return self.update_attitude()
                    
                def update_gps(self):
                    self.prev_lat += 0.00001
                    self.prev_lon += 0.00001
                    return self.prev_lat, self.prev_lon
                    
                def update_gps_safe(self):
                    return self.update_gps()
            
            sensor = MockSensorInterface()

        print("[INFO] 電力センサーインターフェース初期化...")
        try:
            power_sensor = PowerSensorInterface()
        except Exception as e:
            print(f"[ERROR] 電力センサーインターフェース初期化失敗: {e}")
            print("[WARN] モック電力センサーを使用します")
            # モック電力センサーを作成
            class MockPowerSensorInterface:
                def __init__(self):
                    self.voltage = 12.0
                    self.current = 0.0
                    self.power = 0.0
                    self.total_energy_kJ = 0.0
                    print("[INFO] モック電力センサーを初期化しました")
                
                def update(self):
                    # 簡易シミュレーション
                    self.power += 0.1
                    self.total_energy_kJ += 0.01
                
                def get_all(self):
                    return self.voltage, self.current, self.power, self.total_energy_kJ
            
            power_sensor = MockPowerSensorInterface()

        print("[INFO] アクチュエータインターフェース初期化...")
        if DRY_RUN:
            # テストモード用にモックのアクチュエータインターフェースを作成
            class MockActuatorInterface:
                def __init__(self):
                    self.closed_pwm = [350] * 8
                    self.open_pwm = [500] * 8
                    self.mode = None
                    print("[INFO] モックアクチュエータを初期化しました")
                
                def set_keep_mode(self):
                    print("[MOCK] KEEPモードに設定")
                    self.mode = "KEEP"
                    
                def set_straight_mode(self):
                    print("[MOCK] STRAIGHTモードに設定")
                    self.mode = "STRAIGHT"
                    
                def control_thruster(self, direction, pwm):
                    print(f"[MOCK] スラスタ制御: 方向={direction}, PWM={pwm}")
                    
                def stop_thruster(self):
                    print("[MOCK] スラスタ停止")
                    
            actuator = MockActuatorInterface()
        else:
            try:
                actuator = ActuatorInterface(pwm_config_file="servo_pwm_config.csv")
            except Exception as e:
                print(f"[ERROR] アクチュエータインターフェース初期化失敗: {e}")
                print("[WARN] モックアクチュエータを使用します")
                actuator = MockActuatorInterface()
        
        print("[INFO] コントローラ初期化...")
        controller = Controller(pwm_strength=50, distance_tolerance=const.way_point_tolerance)
    
    except Exception as e:
        print(f"[ERROR] 初期化中にエラーが発生しました: {e}")
        print(traceback.format_exc())
        exit(1)

    # --- モード選択 ---
    print("[INFO] モード設定...")
    mode = const.control_mode  # 0: keep, 1: straight

    if mode == 0:
        print("[INFO] KEEPモードを選択")
        try:
            actuator.set_keep_mode()
        except Exception as e:
            print(f"[ERROR] KEEPモード設定中にエラー: {e}")
    elif mode == 1:
        print("[INFO] STRAIGHTモードを選択")
        try:
            actuator.set_straight_mode()
        except Exception as e:
            print(f"[ERROR] STRAIGHTモード設定中にエラー: {e}")

    # --- 定期ログ取得のためのシグナル設定 ---
    print("[INFO] 定期ログ取得を設定...")
    try:
        signal.signal(signal.SIGALRM, logging)
        signal.setitimer(signal.ITIMER_REAL, 0.5, const.timer)
    except Exception as e:
        print(f"[ERROR] シグナル設定中にエラー: {e}")
        print("[WARN] 定期ログ取得が行われない可能性があります")

    # --- メインループ ---
    print("[INFO] メイン制御ループを開始します。Ctrl+Cで終了します。")

    try:
        while True:
            try:
                # 1. センサ更新
                last_lat, last_lon = robot.get_position()
                last_yaw = robot.get_yaw()
                
                try:
                    lat, lon, yaw = safe_get_sensor_data(sensor, 
                                                        default_lat=last_lat, 
                                                        default_lon=last_lon, 
                                                        default_yaw=last_yaw)
                    robot.update_position(lat, lon)
                    robot.update_yaw(yaw)
                except Exception as e:
                    print(f"[ERROR] センサーデータ更新中にエラー: {e}")
                    print("[WARN] 前回の位置・姿勢を使用して処理を継続します")
                    # 前回の値を使用して継続

                # 2. 差分計算
                try:
                    current = np.array([robot.lat, robot.lon])
                    goal = robot.get_current_waypoint()
                    diff_distance = calculator.huveny_distance(goal, current) * 1000  # m
                    target_bearing = math.radians(calculator.calculate_bearing(goal, current))
                    diff_heading = math.degrees(calculator.limit_angle(target_bearing - robot.yaw))

                    robot.diff_distance = diff_distance
                    robot.diff_heading = diff_heading

                    print(f"[INFO] 距離: {diff_distance:.2f} m, 角度差: {diff_heading:.2f} 度, ウェイポイント: {robot.waypoint_num}")
                    
                    # ナビゲーション情報を表示
                    try:
                        display_navigation_guide(
                            current_lat=robot.lat,
                            current_lon=robot.lon,
                            target_lat=goal[0],
                            target_lon=goal[1],
                            yaw=robot.yaw,
                            diff_heading=diff_heading,
                            diff_distance=diff_distance
                        )
                    except Exception as nav_error:
                        print(f"[ERROR] ナビゲーション表示中にエラー: {nav_error}")
                        
                except Exception as e:
                    print(f"[ERROR] 差分計算中にエラー: {e}")
                    print("[WARN] デフォルト値を使用して処理を継続します")
                    # エラー時はデフォルト値を設定して継続
                    robot.diff_distance = 999.9  # 大きな値を設定して移動を促す
                    robot.diff_heading = 0.0

                # 3. 目標に到達しているか確認
                try:
                    if robot.diff_distance < const.way_point_tolerance:
                        print(f"[INFO] ウェイポイント {robot.waypoint_num} に到達しました。")
                        robot.update_waypoint()
                        if robot.get_waypoint_num() == -1:
                            print("[INFO] ミッション完了！すべてのウェイポイントに到達しました。")
                            actuator.stop_thruster()
                            break
                except Exception as e:
                    print(f"[ERROR] ウェイポイント処理中にエラー: {e}")
                    # エラー時は処理を継続
                    
                # 4. 制御コマンドの決定と送信
                try:
                    direction, pwm = controller.decide_command(robot.diff_distance, robot.diff_heading, mode)
                    robot.cmd = direction
                    robot.pwm = pwm
                    
                    # 方向テキストを取得して表示
                    direction_text = direction_to_text(direction, mode)
                    print(f"[COMMAND] 方向: {direction} ({direction_text}), PWM: {pwm}")
                    
                    actuator.control_thruster(direction, pwm)
                except Exception as e:
                    print(f"[ERROR] スラスタ制御中にエラー: {e}")
                    # エラーが発生した場合は安全のためスラスタを停止
                    try:
                        actuator.stop_thruster()
                    except:
                        pass
                    
                    # デフォルト値を設定
                    robot.cmd = 0
                    robot.pwm = 0

                # メインループの待機時間を0.2秒に増やして安定化
                time.sleep(0.2)
            
            except Exception as e:
                print(f"[ERROR] メインループでエラーが発生: {e}")
                print(traceback.format_exc())
                print("[WARN] 5秒後に処理を再開します...")
                time.sleep(5)  # エラー発生時は少し長く待機して再試行

    except KeyboardInterrupt:
        print("[INFO] キーボード割り込みによる停止。")
        try:
            actuator.stop_thruster()
        except Exception as e:
            print(f"[ERROR] スラスタ停止中にエラー: {e}")

    finally:
        # 終了処理・ログ保存
        print("[INFO] 終了処理を実行中...")
        try:
            actuator.stop_thruster()
        except Exception as e:
            print(f"[ERROR] スラスタ停止中にエラー: {e}")
            
        try:
            signal.setitimer(signal.ITIMER_REAL, 0)
        except Exception as e:
            print(f"[ERROR] タイマー停止中にエラー: {e}")
        
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
                print(traceback.format_exc())
                
                # バックアップ試行
                try:
                    backup_file = f'./csv/backup_{date}.csv'
                    with open(backup_file, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(log_header)
                        writer.writerows(log_data)
                    print(f"[INFO] バックアップログを保存しました: {backup_file}")
                except Exception as e:
                    print(f"[ERROR] バックアップログ保存中にもエラー: {e}")
        else:
            print("[INFO] ログデータはありません。")
        
        print("[INFO] テスト終了")

if __name__ == "__main__":
    main()