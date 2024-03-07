import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from actuator_interface import ActuatorInterface

actuator = ActuatorInterface()

try:
    while True:
        cmd = input("方向(0〜18) 強度(0〜100) → 例: '1 50' / 'exit': ").strip()
        if cmd.lower() == 'exit':
            actuator.stop_thruster()
            print("✅ 停止しました。終了。")
            break

        tokens = cmd.split()
        if len(tokens) != 2:
            print("❌ 入力形式エラー")
            continue

        direction = int(tokens[0])
        pwm = int(tokens[1])

        if not (0 <= direction <= 18 and 0 <= pwm <= 100):
            print("❌ 入力範囲エラー")
            continue

        actuator.control_thruster(direction, pwm)

except KeyboardInterrupt:
    actuator.stop_thruster()
    print("✋ 中断されました。スラスタ停止。")
