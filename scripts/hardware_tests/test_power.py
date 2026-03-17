#!/usr/bin/env python3
"""
Power Monitor (INA219) Test Script for Viam Rover 2 (Raspberry Pi 5)

Reads and displays voltage and current from the INA219 power monitor.
Run this BEFORE setting up ROS2 to verify the power monitor works.

I2C Address: 0x40
I2C Bus: 1

Usage:
  python3 test_power.py
  python3 test_power.py --duration 30

Requirements:
  sudo apt install python3-smbus2
"""

import argparse
import time
import sys

try:
    import smbus2
except ImportError:
    print("Error: smbus2 not installed. Run: sudo apt install python3-smbus2")
    sys.exit(1)


# INA219 Register addresses
INA219_ADDR = 0x40
CONFIG_REG = 0x00
SHUNT_VOLTAGE_REG = 0x01
BUS_VOLTAGE_REG = 0x02
POWER_REG = 0x03
CURRENT_REG = 0x04
CALIBRATION_REG = 0x05

# INA219 Configuration
# 32V range, 320mV shunt range, 12-bit, continuous
CONFIG_VALUE = 0x399F

# Calibration value for 0.1 ohm shunt resistor
# Cal = 0.04096 / (Current_LSB * R_shunt)
# With Current_LSB = 0.1mA and R_shunt = 0.1 ohm
# Cal = 0.04096 / (0.0001 * 0.1) = 4096
CALIBRATION_VALUE = 4096
CURRENT_LSB = 0.1  # mA per bit


class INA219:
    def __init__(self, bus_num=1, address=INA219_ADDR, shunt_resistance=0.1):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.shunt_resistance = shunt_resistance

        # Configure the INA219
        self._write_register(CONFIG_REG, CONFIG_VALUE)

        # Set calibration
        self._write_register(CALIBRATION_REG, CALIBRATION_VALUE)

        time.sleep(0.1)
        print(f"INA219 initialized at address 0x{address:02X}")

    def _write_register(self, register, value):
        """Write 16-bit value to register (big-endian)."""
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        self.bus.write_i2c_block_data(self.address, register, [high, low])

    def _read_register(self, register):
        """Read 16-bit value from register (big-endian)."""
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        value = (data[0] << 8) | data[1]
        return value

    def get_bus_voltage(self):
        """Get bus voltage in volts."""
        raw = self._read_register(BUS_VOLTAGE_REG)
        # Voltage is in upper 13 bits, LSB = 4mV
        voltage = (raw >> 3) * 0.004
        return voltage

    def get_shunt_voltage(self):
        """Get shunt voltage in millivolts."""
        raw = self._read_register(SHUNT_VOLTAGE_REG)
        # Convert to signed
        if raw > 32767:
            raw -= 65536
        # LSB = 10µV
        voltage_mv = raw * 0.01
        return voltage_mv

    def get_current(self):
        """Get current in milliamps."""
        raw = self._read_register(CURRENT_REG)
        # Convert to signed
        if raw > 32767:
            raw -= 65536
        current_ma = raw * CURRENT_LSB
        return current_ma

    def get_power(self):
        """Get power in milliwatts."""
        raw = self._read_register(POWER_REG)
        # Power LSB = 20 * Current_LSB
        power_mw = raw * 20 * CURRENT_LSB
        return power_mw

    def get_all(self):
        """Get all measurements."""
        return {
            'voltage': self.get_bus_voltage(),
            'current': self.get_current(),
            'power': self.get_power(),
            'shunt_voltage': self.get_shunt_voltage(),
        }

    def close(self):
        self.bus.close()


def main():
    parser = argparse.ArgumentParser(description='Test INA219 Power Monitor')
    parser.add_argument('--duration', type=int, default=30,
                        help='Test duration in seconds (default: 30)')
    args = parser.parse_args()

    print("="*60)
    print("ROVER2 POWER MONITOR (INA219) TEST")
    print("="*60)

    try:
        ina = INA219()
    except Exception as e:
        print(f"\nERROR: Could not initialize INA219: {e}")
        print("\nTroubleshooting:")
        print("  1. Check I2C is enabled: sudo raspi-config")
        print("  2. Check wiring (SDA, SCL, VCC, GND)")
        print("  3. Run: i2cdetect -y 1 (should show 0x40)")
        sys.exit(1)

    print("\n" + "="*60)
    print("Reading power data...")
    print("Try running motors to see current increase.")
    print("Ctrl+C to stop")
    print("="*60)

    print("\n" + "-"*60)
    print(f"{'Voltage':>12} {'Current':>12} {'Power':>12} {'Shunt V':>12}")
    print(f"{'(V)':>12} {'(mA)':>12} {'(mW)':>12} {'(mV)':>12}")
    print("-"*60)

    min_voltage = float('inf')
    max_voltage = 0
    max_current = 0
    max_power = 0

    try:
        start_time = time.time()

        while time.time() - start_time < args.duration:
            data = ina.get_all()

            voltage = data['voltage']
            current = data['current']
            power = data['power']
            shunt_v = data['shunt_voltage']

            # Track min/max
            min_voltage = min(min_voltage, voltage)
            max_voltage = max(max_voltage, voltage)
            max_current = max(max_current, abs(current))
            max_power = max(max_power, power)

            print(f"\r{voltage:>12.2f} {current:>12.1f} {power:>12.1f} {shunt_v:>12.3f}", end="")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")

    finally:
        ina.close()

    print("\n" + "="*60)
    print("STATISTICS")
    print("="*60)
    print(f"  Voltage range: {min_voltage:.2f}V - {max_voltage:.2f}V")
    print(f"  Max current:   {max_current:.1f} mA")
    print(f"  Max power:     {max_power:.1f} mW")

    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("""
Checklist:
  [ ] Voltage reading is reasonable (12-17V for 4S LiPo)
  [ ] Current reading changes when motors run
  [ ] Power reading is voltage × current

Expected values:
  - Idle: ~12-17V, 100-500mA
  - Motors running: 1000-3000mA depending on load

Battery voltage guide (4S LiPo):
  - 16.8V: Fully charged
  - 14.8V: Nominal
  - 13.2V: Low, charge soon
  - 12.0V: Critical! Stop using immediately

If values are wrong:
  → Check shunt resistor value (default 0.1Ω)
  → Check INA219 wiring
  → Verify battery is connected
""")


if __name__ == '__main__':
    main()
