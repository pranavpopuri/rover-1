#!/usr/bin/env python3
"""
IMU (MPU6050) Test Script for Viam Rover 2 (Raspberry Pi 5)

Reads and displays accelerometer and gyroscope data from the MPU6050.
Run this BEFORE setting up ROS2 to verify the IMU works.

I2C Address: 0x68
I2C Bus: 1

Usage:
  python3 test_imu.py
  python3 test_imu.py --duration 10
  python3 test_imu.py --calibrate

Requirements:
  sudo apt install python3-smbus2
"""

import argparse
import time
import sys
import math

try:
    import smbus2
except ImportError:
    print("Error: smbus2 not installed. Run: sudo apt install python3-smbus2")
    sys.exit(1)


# MPU6050 Register addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
WHO_AM_I = 0x75

# Scale factors
ACCEL_SCALE = 16384.0  # LSB/g for ±2g range
GYRO_SCALE = 131.0     # LSB/(°/s) for ±250°/s range


class MPU6050:
    def __init__(self, bus_num=1, address=MPU6050_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address

        # Check device ID
        who_am_i = self.bus.read_byte_data(self.address, WHO_AM_I)
        if who_am_i != 0x68:
            raise RuntimeError(f"Unexpected WHO_AM_I: 0x{who_am_i:02X} (expected 0x68)")

        # Wake up the MPU6050 (it starts in sleep mode)
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        print(f"MPU6050 initialized at address 0x{address:02X}")

    def read_raw_data(self, register):
        """Read 16-bit signed value from register."""
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def get_accel(self):
        """Get accelerometer data in g."""
        ax = self.read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE
        ay = self.read_raw_data(ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = self.read_raw_data(ACCEL_XOUT_H + 4) / ACCEL_SCALE
        return ax, ay, az

    def get_gyro(self):
        """Get gyroscope data in °/s."""
        gx = self.read_raw_data(GYRO_XOUT_H) / GYRO_SCALE
        gy = self.read_raw_data(GYRO_XOUT_H + 2) / GYRO_SCALE
        gz = self.read_raw_data(GYRO_XOUT_H + 4) / GYRO_SCALE
        return gx, gy, gz

    def get_all(self):
        """Get all sensor data."""
        return self.get_accel(), self.get_gyro()

    def close(self):
        self.bus.close()


def calibrate(imu, samples=100):
    """Calibrate gyroscope by averaging readings while stationary."""
    print("\nCalibrating gyroscope...")
    print("Keep the IMU stationary!")
    time.sleep(1)

    gx_sum, gy_sum, gz_sum = 0, 0, 0

    for i in range(samples):
        gx, gy, gz = imu.get_gyro()
        gx_sum += gx
        gy_sum += gy
        gz_sum += gz
        time.sleep(0.01)
        print(f"\rProgress: {i+1}/{samples}", end="")

    print()

    gx_offset = gx_sum / samples
    gy_offset = gy_sum / samples
    gz_offset = gz_sum / samples

    return gx_offset, gy_offset, gz_offset


def main():
    parser = argparse.ArgumentParser(description='Test MPU6050 IMU')
    parser.add_argument('--duration', type=int, default=30,
                        help='Test duration in seconds (default: 30)')
    parser.add_argument('--calibrate', action='store_true',
                        help='Run gyroscope calibration first')
    args = parser.parse_args()

    print("="*60)
    print("ROVER2 IMU (MPU6050) TEST")
    print("="*60)

    try:
        imu = MPU6050()
    except Exception as e:
        print(f"\nERROR: Could not initialize MPU6050: {e}")
        print("\nTroubleshooting:")
        print("  1. Check I2C is enabled: sudo raspi-config")
        print("  2. Check wiring (SDA, SCL, VCC, GND)")
        print("  3. Run: i2cdetect -y 1 (should show 0x68)")
        sys.exit(1)

    # Calibration
    gyro_offset = (0, 0, 0)
    if args.calibrate:
        gyro_offset = calibrate(imu)
        print(f"\nGyro offsets: X={gyro_offset[0]:.2f}, Y={gyro_offset[1]:.2f}, Z={gyro_offset[2]:.2f} °/s")

    print("\n" + "="*60)
    print("Reading IMU data...")
    print("Try tilting and rotating the rover to see values change.")
    print("Ctrl+C to stop")
    print("="*60)

    print("\n" + "-"*60)
    print(f"{'Accel X':>10} {'Accel Y':>10} {'Accel Z':>10} | {'Gyro X':>10} {'Gyro Y':>10} {'Gyro Z':>10}")
    print(f"{'(g)':>10} {'(g)':>10} {'(g)':>10} | {'(°/s)':>10} {'(°/s)':>10} {'(°/s)':>10}")
    print("-"*60)

    try:
        start_time = time.time()

        while time.time() - start_time < args.duration:
            accel, gyro = imu.get_all()
            ax, ay, az = accel
            gx, gy, gz = gyro

            # Apply calibration offsets
            gx -= gyro_offset[0]
            gy -= gyro_offset[1]
            gz -= gyro_offset[2]

            # Calculate total acceleration (should be ~1g when stationary)
            total_accel = math.sqrt(ax*ax + ay*ay + az*az)

            print(f"\r{ax:>10.3f} {ay:>10.3f} {az:>10.3f} | {gx:>10.2f} {gy:>10.2f} {gz:>10.2f}  (|a|={total_accel:.3f}g)", end="")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")

    finally:
        imu.close()

    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("""
Checklist:
  [ ] Accelerometer shows ~0, 0, 1g when flat (Z up)
  [ ] Tilting changes X and Y acceleration
  [ ] Gyroscope shows ~0 when stationary
  [ ] Rotating shows gyro values change
  [ ] Total acceleration |a| ≈ 1.0g when stationary

Expected values (flat, stationary):
  Accel: X≈0, Y≈0, Z≈1.0 (±0.1g)
  Gyro:  X≈0, Y≈0, Z≈0 (±2°/s without calibration)

If values are wrong:
  → Check mounting orientation
  → Run with --calibrate for better gyro readings
  → Check for vibration or movement during test
""")


if __name__ == '__main__':
    main()
