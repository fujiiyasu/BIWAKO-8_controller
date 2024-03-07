# robot.py

class Robot:
    def __init__(self, waypoint_list):
        # 初期状態
        self.lon = 0.0
        self.lat = 0.0
        self.yaw = 0.0

        self.waypoint_list = waypoint_list
        self.waypoint_num = 0
        self.current_waypoint = self.waypoint_list[self.waypoint_num]

        self.voltage = 0.0
        self.current = 0.0
        self.power = 0.0
        self.consumed_energy = 0.0

        self.speed = 0.0
        self.diff_distance = 0.0
        self.diff_heading = 0.0

        self.cmd = 0
        self.pwm = 1500
        self.count = 0.0

    # --- 位置・姿勢情報の更新と取得 ---
    def update_position(self, lat, lon):
        self.lat = lat
        self.lon = lon

    def update_yaw(self, yaw):
        self.yaw = yaw

    def get_position(self):
        return [self.lat, self.lon]

    def get_yaw(self):
        return self.yaw

    # --- エネルギー関連 ---
    def update_voltage(self, v):
        self.voltage = v

    def update_current(self, c):
        self.current = c

    def update_power(self, p):
        self.power = p
        self.consumed_energy += p / 1000.0  # kJに換算して蓄積

    def get_energy_status(self):
        return self.voltage, self.current, self.power, self.consumed_energy

    # --- ウェイポイント関連 ---
    def get_current_waypoint(self):
        return self.current_waypoint

    def get_waypoint_num(self):
        return self.waypoint_num

    def update_waypoint(self):
        if self.waypoint_num < len(self.waypoint_list) - 1:
            self.waypoint_num += 1
            self.current_waypoint = self.waypoint_list[self.waypoint_num]
        else:
            self.waypoint_num = -1
            print("All waypoints reached!")

    # --- ログ・制御情報 ---
    def count_up(self, step=0.1):
        self.count += step

    def get_status_summary(self):
        return {
            "position": self.get_position(),
            "yaw": self.yaw,
            "waypoint": self.current_waypoint,
            "energy": self.get_energy_status(),
            "diff_distance": self.diff_distance,
            "diff_heading": self.diff_heading,
            "count": self.count,
        }
