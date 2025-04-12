# calculate_degree.py

import math
import numpy as np

# === Hubeny式による距離計算 ===
def huveny_distance(target_point, current_point):
    e_radius = 6378137.0              # 地球赤道半径 [m]
    p_radius = 6356752.314140         # 地球極半径 [m]

    # 差分をラジアンに変換！
    dy = math.radians(target_point[0] - current_point[0])
    dx = math.radians(target_point[1] - current_point[1])
    my = math.radians((target_point[0] + current_point[0]) / 2)

    e = math.sqrt((e_radius**2 - p_radius**2) / e_radius**2)
    W = math.sqrt(1 - (e * math.sin(my))**2)
    M = e_radius * (1 - e**2) / W**3
    N = e_radius / W

    d = math.sqrt((dy * M)**2 + (dx * N * math.cos(my))**2)
    return d / 1000  # [km]

# === Haversine式による方位角計算 ===
"""
def calculate_bearing(target_point, current_point):
    lat1 = math.radians(target_point[0])
    lat2 = math.radians(current_point[0])
    lon1 = math.radians(target_point[1])
    lon2 = math.radians(current_point[1])
    dlon = lon2 - lon1
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(y, x))
    bearing = (bearing + 360) % 360
    return bearing
"""
def calculate_bearing(target_point, current_point):
    # target_pointは[lat, lon]、current_pointも[lat, lon]
    lat1 = math.radians(current_point[0])  # 現在の緯度
    lat2 = math.radians(target_point[0])   # 目標の緯度
    lon1 = math.radians(current_point[1])  # 現在の経度
    lon2 = math.radians(target_point[1])   # 目標の経度
    
    dlon = lon2 - lon1
    
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    
    bearing = math.atan2(y, x)
    bearing_deg = math.degrees(bearing)
    bearing_deg = (bearing_deg + 360) % 360  # 0-360の範囲に正規化
    
    return bearing

# === -π ～ π に角度を制限 ===
"""
def limit_angle(angle_in):
    angle_out = angle_in
    while angle_out > math.pi:
        angle_out -= 2 * math.pi
    while angle_out < -math.pi:
        angle_out += 2 * math.pi
    return angle_out
"""
def limit_angle(angle_in):
    angle_out = angle_in
    while angle_out > math.pi:
        angle_out -= 2 * math.pi
    while angle_out < -math.pi:
        angle_out += 2 * math.pi
    return angle_out

# === 度 → ラジアン ===
def translate_deg_to_rad(deg_val):
    return deg_val * math.pi / 180

# === [lat, lon] in degrees → radians ===
def translate_decimal_to_rad(decimal_val):
    return [translate_deg_to_rad(decimal_val[0]), translate_deg_to_rad(decimal_val[1])]

# === 度分秒表記（dddmm.mmmm）→ 十進数 ===
def translate_sexagesimal_to_decimal(degval):
    decimal, integer = math.modf(degval / 100)
    decimal_val = integer + decimal / 60.0 * 100.0
    return decimal_val

# === GPGGA形式（[lon, lat]）→ decimal座標 ===
def translate_GPGGA_to_decimal(GPGGAval):
    decimal_longtitude = translate_sexagesimal_to_decimal(GPGGAval[0])
    decimal_latitude = translate_sexagesimal_to_decimal(GPGGAval[1])
    return [decimal_longtitude, decimal_latitude]
