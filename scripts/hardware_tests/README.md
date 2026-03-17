# Hardware Test Scripts

Standalone Python scripts to test Rover2 hardware on Raspberry Pi 5 **without ROS2**.

Run these BEFORE setting up ROS2 to verify your hardware works.

## Prerequisites

```bash
# Install dependencies
sudo apt update
sudo apt install -y python3-lgpio python3-smbus2 i2c-tools

# Enable I2C
sudo raspi-config
# → Interface Options → I2C → Enable
# Reboot after enabling

# Add user to gpio group (for non-root access)
sudo usermod -aG gpio $USER
# Logout and login again
```

## Test Order

Run tests in this order:

### 1. I2C Devices
```bash
python3 test_i2c.py
```
Checks that IMU (0x68) and power monitor (0x40) are detected.

### 2. IMU (MPU6050)
```bash
python3 test_imu.py
python3 test_imu.py --calibrate  # With gyro calibration
```
Reads accelerometer and gyroscope. Tilt the rover to see values change.

### 3. Power Monitor (INA219)
```bash
python3 test_power.py
```
Reads battery voltage and current. Run motors to see current increase.

### 4. Motors
```bash
python3 test_motors.py
python3 test_motors.py --speed 50  # Higher speed
python3 test_motors.py --motor left  # Test one motor
```
**WARNING:** Lift the rover first! Wheels will spin.

### 5. Encoders
```bash
python3 test_encoders.py
```
Manually rotate wheels and verify tick counts (~1992 per rotation).

## GPIO Pin Reference

| Function | Physical Pin | BCM GPIO |
|----------|-------------|----------|
| Left Motor PWM | 37 | 26 |
| Left Motor A | 35 | 19 |
| Left Motor B | 33 | 13 |
| Right Motor PWM | 15 | 22 |
| Right Motor A | 31 | 6 |
| Right Motor B | 29 | 5 |
| Left Encoder | 38 | 20 |
| Right Encoder | 40 | 21 |
| I2C SDA | 3 | 2 |
| I2C SCL | 5 | 3 |

## Troubleshooting

### "Permission denied" on GPIO
```bash
sudo usermod -aG gpio $USER
# Then logout and login
```

### I2C devices not found
```bash
# Check I2C is enabled
ls /dev/i2c*
# Should show /dev/i2c-1

# Scan for devices
i2cdetect -y 1
# Should show 40 and 68
```

### Motors don't spin
- Check battery charge
- Verify motor wiring
- Try higher speed: `--speed 50`

### Encoder count wrong
- Verify encoder wiring
- Check encoder power (3.3V or 5V)
- Note actual ticks/rotation for config
