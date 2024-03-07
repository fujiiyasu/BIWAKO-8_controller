import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
from actuator_interface import ActuatorInterface

actuator = ActuatorInterface(pwm_config_file="servo_params.csv")

try:
    while True:
        print("[TEST] Closed Mode: 偶数→奇数")
        for ch in range(0, 8, 2):
            actuator.link_pwm.set_pwm(ch, 0, actuator.closed_pwm[ch])
        time.sleep(1)
        for ch in range(1, 8, 2):
            actuator.link_pwm.set_pwm(ch, 0, actuator.closed_pwm[ch])
        time.sleep(2)

        print("[TEST] Open Mode: 偶数→奇数")
        for ch in range(0, 8, 2):
            actuator.link_pwm.set_pwm(ch, 0, actuator.open_pwm[ch])
        time.sleep(1)
        for ch in range(1, 8, 2):
            actuator.link_pwm.set_pwm(ch, 0, actuator.open_pwm[ch])
        time.sleep(2)

except KeyboardInterrupt:
    print("⛔ 停止しましたがサーボ状態はそのまま維持されます。")
