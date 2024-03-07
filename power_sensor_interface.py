# power_sensor_interface.py

from INA226 import INA226

class PowerSensorInterface:
    def __init__(self, i2c_address=0x40):
        self.sensor = INA226(i2c_address)
        self.sensor.initial_operation()

        self.voltage = 0.0
        self.current = 0.0
        self.power = 0.0
        self.total_energy_kJ = 0.0  # 積算電力量[kJ]

    def update(self):
        """
        電圧・電流・電力を取得し、蓄積エネルギーも更新する。
        """
        self.voltage = self.sensor.get_voltage()
        self.current = self.sensor.get_current()
        self.power = self.sensor.get_power()

        self.total_energy_kJ += self.power / 1000.0  # mW → kJ換算

    def get_voltage(self):
        return self.voltage

    def get_current(self):
        return self.current

    def get_power(self):
        return self.power

    def get_total_energy(self):
        return self.total_energy_kJ

    def get_all(self):
        """
        まとめて取得（ログ用など）
        :return: (voltage[V], current[A], power[mW], total_energy[kJ])
        """
        return self.voltage, self.current, self.power, self.total_energy_kJ
