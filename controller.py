# controller.py

class Controller:
    def __init__(self, pwm_strength=50, distance_tolerance=1.0):
        self.pwm_strength = pwm_strength
        self.distance_tolerance = distance_tolerance

        # 前回のコマンド（不要なら削除可能）
        self.prev_cmd_flag = -1

    def decide_command(self, diff_distance, diff_deg, mode):
        """
        距離と方位差から推進方向を決定

        Parameters:
            diff_distance: float 距離 [m]
            diff_deg: float 角度差 [-180 ~ 180]
            mode: int モード（0: keep, 1: straight）

        Returns:
            (direction, pwm): (int, int)
        """
        if diff_distance < self.distance_tolerance:
            return 0, 0  # 停止

        # 初期値（停止）
        direction = 0

        if mode == 0:
            # --- KEEP モード（4方向） ---
            if -45.0 <= diff_deg < 45:
                direction = 1  # 前進
            elif -180.0 <= diff_deg < -135.0 or 135.0 <= diff_deg < 180.0:
                direction = 2  # 後退
            elif 45.0 <= diff_deg < 135.0:
                direction = 3  # 右
            elif -135.0 <= diff_deg < -45.0:
                direction = 4  # 左

        elif mode == 1:
            # --- STRAIGHT モード（8方向） ---
            if -30.0 <= diff_deg < 30:
                direction = 1  # 前進
            elif -180.0 <= diff_deg < -150.0 or 150.0 <= diff_deg < 180.0:
                direction = 2  # 後退
            elif 30.0 <= diff_deg < 90.0:
                direction = 3  # 前右斜め（左回転気味）
            elif 90.0 <= diff_deg < 150.0:
                direction = 4  # 後右斜め
            elif -150.0 <= diff_deg < -90.0:
                direction = 5  # 後左斜め
            elif -90.0 <= diff_deg < -30.0:
                direction = 6  # 前左斜め（右回転気味）

        return direction, self.pwm_strength
