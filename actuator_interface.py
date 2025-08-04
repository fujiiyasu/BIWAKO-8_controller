# actuator_interface.py

import time
import csv
import Adafruit_PCA9685
from smbus import SMBus

class ActuatorInterface:
    def __init__(self, arduino_addr=0x10, link_pwm_addr=0x41, pwm_config_file="servo_pwm_config.csv"):
        # I2C通信設定
        self.arduino = arduino_addr
        self.i2cbus = SMBus(6)

        # PWM制御設定
        self.link_pwm = Adafruit_PCA9685.PCA9685(link_pwm_addr)
        self.link_pwm.set_pwm_freq(60)

        # PWM設定ファイルから読み込み
        self.closed_pwm = [0] * 8
        self.open_pwm = [0] * 8
        self._load_pwm_config(pwm_config_file)

    def _load_pwm_config(self, filepath):
        try:
            with open(filepath, mode='r', encoding='utf-8') as file:
                reader = csv.reader(file)
                for row in reader:
                    if not row or row[0].startswith('#'):
                        continue
                    idx = int(row[0])
                    self.closed_pwm[idx] = int(row[1])
                    self.open_pwm[idx] = int(row[2])
            print(f"[INFO] Loaded PWM config from {filepath}")
        except Exception as e:
            print(f"[ERROR] Failed to load PWM config: {e}")

    # --- スラスタ制御 ---
    def control_thruster(self, direction, pwm_strength):
        self.i2cbus.write_i2c_block_data(self.arduino, 2, [direction, pwm_strength])

    def stop_thruster(self):
        self.i2cbus.write_i2c_block_data(self.arduino, 2, [0, 0])

    # --- サーボ動作：開/閉 ---
    def set_open_mode(self):
        print("[Actuator] Open Mode")
        for ch in range(8):
            self.link_pwm.set_pwm(ch, 0, self.open_pwm[ch])
        time.sleep(1)

    def set_closed_mode(self):
        print("[Actuator] Closed Mode")
        for ch in range(8):
            self.link_pwm.set_pwm(ch, 0, self.closed_pwm[ch])
        time.sleep(1)

    # --- モード変形（修正版） ---
    def set_straight_mode(self):
        print("[Actuator] Set STRAIGHT Mode (CLOSE)")
        self.set_closed_mode()

    def set_keep_mode(self):
        print("[Actuator] Set KEEP Mode (OPEN)")
        self.set_open_mode()

    def disarm(self):
        print("[Actuator] Disarm")
        self.stop_thruster()
