# tests/test_calculate_degree.py
# tests/test_robot.py の冒頭に追加
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import math
from calculate_degree import (
    huveny_distance, calculate_bearing, limit_angle,
    translate_deg_to_rad, translate_decimal_to_rad,
    translate_sexagesimal_to_decimal, translate_GPGGA_to_decimal
)

def test_huveny_distance():
    p1 = [35.0, 135.0]
    p2 = [35.001, 135.001]
    d = huveny_distance(p1, p2)
    print(d)
    assert 0 < d < 0.2  # km 単位（およそ 150m 程度）
    print(f"✅ huveny_distance: {d*1000:.2f} m")

def test_calculate_bearing():
    p1 = [35.0, 135.0]
    p2 = [35.001, 135.0]  # 真北
    bearing = calculate_bearing(p1, p2)
    assert 0 <= bearing <= 360
    assert abs(bearing - 0.0) < 10  # 真北に近い
    print(f"✅ calculate_bearing: {bearing:.2f} deg")

def test_limit_angle():
    assert abs(limit_angle(math.radians(190)) - math.radians(-170)) < 1e-5
    assert abs(limit_angle(math.radians(-190)) - math.radians(170)) < 1e-5
    print("✅ limit_angle passed")

def test_translate_deg_to_rad():
    assert abs(translate_deg_to_rad(180) - math.pi) < 1e-5
    print("✅ translate_deg_to_rad passed")

def test_translate_decimal_to_rad():
    result = translate_decimal_to_rad([180, 90])
    assert len(result) == 2
    assert abs(result[0] - math.pi) < 1e-5
    assert abs(result[1] - math.pi / 2) < 1e-5
    print("✅ translate_decimal_to_rad passed")

def test_translate_sexagesimal_to_decimal():
    dms = 3507.0000  # 35度7分 → 35.116666...
    dec = translate_sexagesimal_to_decimal(dms)
    assert abs(dec - 35.1166) < 0.01
    print("✅ translate_sexagesimal_to_decimal passed")

def test_translate_GPGGA_to_decimal():
    gpgga = [13507.1234, 3507.5678]
    result = translate_GPGGA_to_decimal(gpgga)
    assert len(result) == 2
    assert abs(result[0] - 135.1187) < 0.01
    assert abs(result[1] - 35.1261) < 0.01
    print("✅ translate_GPGGA_to_decimal passed")

if __name__ == "__main__":
    test_huveny_distance()
    test_calculate_bearing()
    test_limit_angle()
    test_translate_deg_to_rad()
    test_translate_decimal_to_rad()
    test_translate_sexagesimal_to_decimal()
    test_translate_GPGGA_to_decimal()
