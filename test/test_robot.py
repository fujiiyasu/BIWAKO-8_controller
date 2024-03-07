# tests/test_robot.py の冒頭に追加
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot import Robot

def test_robot_state_update():
    wp = [[35.0, 135.0], [35.01, 135.01]]
    robot = Robot(wp)

    # 初期値確認
    assert robot.get_current_waypoint() == [35.0, 135.0]

    # 更新テスト
    robot.update_position(35.05, 135.05)
    assert robot.lat == 35.05
    assert robot.lon == 135.05

    robot.update_yaw(1.57)
    assert abs(robot.yaw - 1.57) < 1e-5

    print("✅ Robot クラスのテスト成功")

if __name__ == "__main__":
    test_robot_state_update()