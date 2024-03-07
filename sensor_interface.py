# sensor_interface.py

from pymavlink import mavutil

class SensorInterface:
    def __init__(self, connection_string='udp:127.0.0.1:14551'):
        self.master = mavutil.mavlink_connection(connection_string)
        self.wait_heartbeat()
        print("[INFO] MAVLink: Heartbeat received.")

        # --- 安全な初期値（例：琵琶湖中央） ---
        self.prev_lat = 35.123456
        self.prev_lon = 135.123456
        self.prev_yaw = 0.0

    def wait_heartbeat(self):
        print("[INFO] Waiting for heartbeat...")
        self.master.wait_heartbeat()

    def update_attitude(self):
        attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=True)
        if attitude_msg:
            yaw = float(attitude_msg.to_dict()['yaw'])
            self.prev_yaw = yaw
            return yaw
        else:
            print("[WARN] Failed to get ATTITUDE. Using previous yaw.")
            return self.prev_yaw

    def update_gps(self):
        gps_msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if gps_msg:
            data = gps_msg.to_dict()
            lat = float(data['lat']) / 1e7
            lon = float(data['lon']) / 1e7
            self.prev_lat = lat
            self.prev_lon = lon
            return lat, lon
        else:
            print("[WARN] Failed to get GPS. Using previous lat/lon.")
            return self.prev_lat, self.prev_lon

    def get_sensor_data(self):
        lat, lon = self.update_gps()
        yaw = self.update_attitude()
        return lat, lon, yaw
