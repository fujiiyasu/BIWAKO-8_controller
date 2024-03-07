# tests/test_robot.py の冒頭に追加
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from power_sensor_interface import PowerSensorInterface

def test_power_sensor():
    power = PowerSensorInterface()
    power.update()

    v, c, p, e = power.get_all()

    assert v > 0
    assert c >= 0
    assert p >= 0

    print(f"✅ Power OK: {v:.2f} V, {c:.2f} A, {p:.2f} mW, {e:.3f} kJ")

if __name__ == "__main__":
    test_power_sensor()