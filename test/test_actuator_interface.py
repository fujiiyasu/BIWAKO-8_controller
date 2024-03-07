# tests/test_robot.py の冒頭に追加
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from actuator_interface import ActuatorInterface
import time

def test_actuator():
    actuator = ActuatorInterface(pwm_config_file="servo_pwm_config.csv")

    print("🌀 開く")
    actuator.set_open_mode()
    time.sleep(1)

    print("🌀 閉じる")
    actuator.set_closed_mode()
    time.sleep(1)

    print("🌀 スラスタ制御")
    actuator.control_thruster(1, 50)
    time.sleep(1)
    actuator.stop_thruster()

    print("✅ Actuator テスト完了")

if __name__ == "__main__":
    test_actuator()