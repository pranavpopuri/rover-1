#!/usr/bin/env python3
"""
I2C Device Test Script for Viam Rover 2 (Raspberry Pi 5)

Scans the I2C bus and checks for expected devices:
  - MPU6050 IMU at address 0x68
  - INA219 Power Monitor at address 0x40

Usage:
  python3 test_i2c.py

Requirements:
  sudo apt install i2c-tools python3-smbus2

Make sure I2C is enabled:
  sudo raspi-config → Interface Options → I2C → Enable
"""

import sys
import subprocess


def run_i2cdetect():
    """Run i2cdetect and return the output."""
    try:
        result = subprocess.run(
            ['i2cdetect', '-y', '1'],
            capture_output=True,
            text=True,
            timeout=5
        )
        return result.stdout, result.returncode
    except FileNotFoundError:
        return None, -1
    except subprocess.TimeoutExpired:
        return None, -2


def parse_i2cdetect(output):
    """Parse i2cdetect output to find detected addresses."""
    addresses = []
    for line in output.split('\n'):
        if ':' in line:
            parts = line.split(':')[1].strip().split()
            for part in parts:
                if part != '--' and part != 'UU':
                    try:
                        addr = int(part, 16)
                        addresses.append(addr)
                    except ValueError:
                        pass
    return addresses


def main():
    print("="*60)
    print("ROVER2 I2C DEVICE TEST")
    print("="*60)

    # Expected devices
    expected = {
        0x40: ('INA219', 'Power Monitor'),
        0x68: ('MPU6050', 'IMU (Accelerometer/Gyroscope)'),
    }

    print("\nExpected devices:")
    for addr, (name, desc) in expected.items():
        print(f"  0x{addr:02X}: {name} - {desc}")

    print("\nScanning I2C bus 1...")
    print("-"*60)

    output, code = run_i2cdetect()

    if code == -1:
        print("ERROR: i2cdetect not found!")
        print("Install with: sudo apt install i2c-tools")
        sys.exit(1)
    elif code == -2:
        print("ERROR: i2cdetect timed out")
        sys.exit(1)
    elif code != 0:
        print(f"ERROR: i2cdetect failed with code {code}")
        print("Make sure I2C is enabled:")
        print("  sudo raspi-config → Interface Options → I2C → Enable")
        sys.exit(1)

    print(output)

    # Parse detected addresses
    detected = parse_i2cdetect(output)

    print("-"*60)
    print("RESULTS")
    print("-"*60)

    all_found = True
    for addr, (name, desc) in expected.items():
        if addr in detected:
            print(f"  [OK]  0x{addr:02X} {name}: DETECTED")
        else:
            print(f"  [FAIL] 0x{addr:02X} {name}: NOT FOUND")
            all_found = False

    # Report any unexpected devices
    unexpected = [a for a in detected if a not in expected]
    if unexpected:
        print("\nUnexpected devices found:")
        for addr in unexpected:
            print(f"  0x{addr:02X}: Unknown device")

    print("\n" + "="*60)
    if all_found:
        print("SUCCESS: All expected I2C devices found!")
    else:
        print("WARNING: Some devices not found")
        print("""
Troubleshooting:
  1. Check I2C is enabled:
     sudo raspi-config → Interface Options → I2C → Enable

  2. Check wiring:
     - SDA to GPIO 2 (Pin 3)
     - SCL to GPIO 3 (Pin 5)
     - VCC to 3.3V or 5V (check device specs)
     - GND to Ground

  3. Check for shorts or loose connections

  4. Try rebooting: sudo reboot
""")
    print("="*60)

    return 0 if all_found else 1


if __name__ == '__main__':
    sys.exit(main())
