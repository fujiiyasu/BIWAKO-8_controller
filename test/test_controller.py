# tests/test_robot.py の冒頭に追加
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from controller import Controller

def test_controller_logic():
    ctl = Controller(pwm_strength=40)

    # 前進すべき
    dir1, pwm1 = ctl.decide_command(2.0, 0, mode=0)
    assert dir1 == 1

    # 後退すべき
    dir2, pwm2 = ctl.decide_command(3.0, 170, mode=0)
    assert dir2 == 2

    print("✅ Controller 指令: 前進→", dir1, " 後退→", dir2)

if __name__ == "__main__":
    test_controller_logic()