# tests/test_robot.py の冒頭に追加
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sensor_interface import SensorInterface

def test_sensor_fetch():
    sensor = SensorInterface()
    lat, lon, yaw = sensor.get_sensor_data()

    assert lat is not None
    assert lon is not None
    assert yaw is not None

    print(f"✅ GPS: ({lat:.6f}, {lon:.6f}), Yaw: {yaw:.3f} rad")

if __name__ == "__main__":
    test_sensor_fetch()