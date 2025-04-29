# main.py

import time
import numpy as np
import math
import signal
import csv
import traceback
from datetime import datetime
import os
import sys

# カレントディレクトリが正しく設定されていることを確認
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

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
def heading_to_direction_indicator(diff_heading, mode=1):
    """
    方位角の差から進行方向を矢印で表示する
    
    Args:
        diff_heading: 目標方位と現在方位の差（度）-180〜180の範囲
        mode: 移動モード (0: キープモード(4方向), 1: ストレートモード(8方向))
    
    Returns:
        方向を示す矢印の文字列
    """
    # キープモード: 4方向表示
    if mode == 0:
        if -45.0 <= diff_heading < 45.0:
            return "↑ (前)"
        elif 45.0 <= diff_heading < 135.0:
            return "→ (右)"
        elif -135.0 <= diff_heading < -45.0:
            return "← (左)"
        elif diff_heading >= 135.0 or diff_heading < -135.0:
            return "↓ (後)"
        else:
            return "? (不明)"
    
    # ストレートモード: 8方向表示
    else:
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

# --- シンプル版ナビゲーション情報表示 ---
def display_simple_navigation_info(current_lat, current_lon, target_lat, target_lon, yaw, diff_heading, diff_distance, mode):
    """
    シンプルなナビゲーション情報を表示（stdout再入問題を回避）
    """
    current_heading_deg = math.degrees(yaw)
    direction_indicator = heading_to_direction_indicator(diff_heading, mode)
    print(f"現在位置: 緯度 {current_lat:.6f}°, 経度 {current_lon:.6f}°")
    print(f"目標地点: 緯度 {target_lat:.6f}°, 経度 {target_lon:.6f}°")
    print(f"現在方位: {current_heading_deg:.1f}°")
    print(f"方位差  : {diff_heading:.1f}°")
    print(f"進行方向: {direction_indicator}")
    print(f"残り距離: {diff_distance:.2f} m")

# --- 安全にセンサーデータを取得する関数 ---
def safe_get_sensor_data(sensor, default_lat=35.0, default_lon=135.0, default_yaw=0.0):
    try:
        # 安全なメソッドを使用（修正版SensorInterfaceが実装している場合）
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

# --- ログ保存ディレクトリの作成 ---
if not os.path.exists('./csv'):
    os.makedirs('./csv')

# --- グローバル変数 ---
log_data = []
log_header = ['count', 'latitude', 'longitude', 'yaw', 'cmd', 'pwm',
              'voltage', 'current', 'power', 'energy', 'distance', 'heading_diff', 'waypoint_num']
date = time.strftime("%Y%m%d%H%M%S")

# --- 割り込み用ログ関数 ---
def logging(signum=None, frame=None):
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
            # エラーがあっても継続

        # === 2. 電力センサの取得 ===
        try:
            v, c, p, e = safe_get_power_data(power_sensor)
            robot.update_voltage(v)
            robot.update_current(c)
            robot.update_power(p)
        except Exception as e:
            print(f"[ERROR] 電力センサー更新中にエラー: {e}")
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
        except Exception as e:
            print(f"[ERROR] ログデータ作成中にエラー: {e}")
            # エラーがあっても継続

    except Exception as e:
        print(f"[ERROR] ロギング処理全体でエラー: {e}")
        # ロギングでエラーが発生しても処理は継続させる

# --- メインループ ---
if __name__ == "__main__":
    try:
        print("[INFO] 初期化中...")
        
        # --- 初期設定 ---
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
            raise e  # 本番環境なのでセンサー初期化失敗は致命的エラー

        print("[INFO] 電力センサーインターフェース初期化...")
        try:
            power_sensor = PowerSensorInterface()
        except Exception as e:
            print(f"[ERROR] 電力センサーインターフェース初期化失敗: {e}")
            raise e  # 本番環境なので電力センサー初期化失敗は致命的エラー

        print("[INFO] アクチュエータインターフェース初期化...")
        try:
            actuator = ActuatorInterface(pwm_config_file="servo_pwm_config.csv")
        except Exception as e:
            print(f"[ERROR] アクチュエータインターフェース初期化失敗: {e}")
            raise e  # 本番環境なのでアクチュエータ初期化失敗は致命的エラー
        
        print("[INFO] コントローラ初期化...")
        controller = Controller(pwm_strength=50, distance_tolerance=const.way_point_tolerance)

        # --- モード選択 ---
        print("[INFO] モード設定...")
        mode = const.control_mode  # 0: keep, 1: straight

        if mode == 0:
            print("[INFO] KEEPモードを選択")
            actuator.set_keep_mode()
        elif mode == 1:
            print("[INFO] STRAIGHTモードを選択")
            actuator.set_straight_mode()

        # --- 定期ログ取得のためのシグナル設定 ---
        print("[INFO] 定期ログ取得を設定...")
        signal.signal(signal.SIGALRM, logging)
        signal.setitimer(signal.ITIMER_REAL, 0.5, const.timer)

        print("[INFO] メイン制御ループを開始します。Ctrl+Cで終了します。")
        
        # --- メインループ ---
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
                    
                    # シンプル版ナビゲーション情報表示（再入問題を回避）
                    try:
                        display_simple_navigation_info(
                            current_lat=robot.lat,
                            current_lon=robot.lon,
                            target_lat=goal[0],
                            target_lon=goal[1],
                            yaw=robot.yaw,
                            diff_heading=diff_heading,
                            diff_distance=diff_distance,
                            mode=mode
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

                # メインループの待機時間（リアルタイムコントロールのバランス）
                time.sleep(0.03)
            
            except Exception as e:
                print(f"[ERROR] メインループでエラーが発生: {e}")
                print(traceback.format_exc())
                
                # 5秒後に処理を再開（再試行）
                print("[WARN] 5秒後に処理を再開します...")
                time.sleep(5)

    except KeyboardInterrupt:
        print("[INFO] キーボード割り込みによる停止。")
        actuator.stop_thruster()

    except Exception as e:
        print(f"[ERROR] 予期せぬエラーが発生: {e}")
        print(traceback.format_exc())
        try:
            actuator.stop_thruster()
        except:
            pass

    finally:
        # 終了処理・ログ保存
        print("[INFO] 終了処理を実行中...")
        try:
            actuator.stop_thruster()
            actuator.i2cbus.write_i2c_block_data(actuator.arduino, 0, [0, 0])
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
        
        print("[INFO] プログラム終了")