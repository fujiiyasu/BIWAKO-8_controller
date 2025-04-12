# sensor_interface.py

from pymavlink import mavutil
import time

class SensorInterface:
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        try:
            self.master = mavutil.mavlink_connection(connection_string)
            self.wait_heartbeat()
            print("[INFO] MAVLink: Heartbeat received.")

            # --- 安全な初期値（例：琵琶湖中央） ---
            self.prev_lat = 35.123456
            self.prev_lon = 135.123456
            self.prev_yaw = 0.0
            
            # --- 最終更新時刻 ---
            self.last_attitude_update = time.time()
            self.last_gps_update = time.time()
            
            # --- デバッグフラグ ---
            self.debug = False
            
        except Exception as e:
            print(f"[ERROR] SensorInterface初期化エラー: {e}")
            raise e

    def wait_heartbeat(self):
        """
        MAVLinkハートビートを待機（タイムアウト付き）
        """
        print("[INFO] Waiting for heartbeat...")
        start_time = time.time()
        while time.time() - start_time < 10.0:  # 10秒でタイムアウト
            try:
                self.master.wait_heartbeat(timeout=1.0)
                return True
            except Exception as e:
                if self.debug:
                    print(f"[WARN] ハートビート待機中...: {e}")
        
        print("[ERROR] ハートビート待機タイムアウト")
        raise TimeoutError("ハートビート待機がタイムアウトしました")

    # ----- 非ブロッキング方式のデータ取得メソッド -----
    
    def update_attitude_nonblocking(self):
        """
        非ブロッキングでアティチュードを更新
        """
        try:
            # タイムアウトを設定してBlockingにならないようにする
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                yaw = float(attitude_msg.to_dict()['yaw'])
                self.prev_yaw = yaw
                self.last_attitude_update = time.time()
                return yaw
            
            # 一定時間データが更新されていない場合は警告
            if time.time() - self.last_attitude_update > 5.0 and self.debug:
                print("[WARN] 5秒以上姿勢データが更新されていません。前回値を使用します。")
                
            return self.prev_yaw
        except Exception as e:
            if self.debug:
                print(f"[ERROR] 姿勢更新エラー: {e}")
            return self.prev_yaw

    def update_gps_nonblocking(self):
        """
        非ブロッキングでGPSデータを更新
        """
        try:
            gps_msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if gps_msg:
                data = gps_msg.to_dict()
                lat = float(data['lat']) / 1e7
                lon = float(data['lon']) / 1e7
                self.prev_lat = lat
                self.prev_lon = lon
                self.last_gps_update = time.time()
                return lat, lon
            
            # 一定時間データが更新されていない場合は警告
            if time.time() - self.last_gps_update > 5.0 and self.debug:
                print("[WARN] 5秒以上GPSデータが更新されていません。前回値を使用します。")
                
            return self.prev_lat, self.prev_lon
        except Exception as e:
            if self.debug:
                print(f"[ERROR] GPS更新エラー: {e}")
            return self.prev_lat, self.prev_lon

    # ----- 安全なデータ取得メソッド（新規追加） -----
    
    def update_attitude_safe(self):
        """
        姿勢を安全に取得（再帰呼び出しなし）
        """
        return self.update_attitude_nonblocking()
        
    def update_gps_safe(self):
        """
        GPSデータを安全に取得（再帰呼び出しなし）
        """
        return self.update_gps_nonblocking()
        
    def get_sensor_data_safe(self):
        """
        全センサーデータを安全に取得（再帰呼び出しなし）
        """
        lat, lon = self.update_gps_nonblocking()
        yaw = self.update_attitude_nonblocking()
        return lat, lon, yaw
    
    # ----- 互換性のための既存メソッド（オーバーライド） -----
    
    def update_attitude(self):
        """
        元のメソッドを安全なバージョンでオーバーライド
        """
        return self.update_attitude_safe()

    def update_gps(self):
        """
        元のメソッドを安全なバージョンでオーバーライド
        """
        return self.update_gps_safe()

    def get_sensor_data(self):
        """
        元のメソッドを安全なバージョンでオーバーライド
        """
        return self.get_sensor_data_safe()