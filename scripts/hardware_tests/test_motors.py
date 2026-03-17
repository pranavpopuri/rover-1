#!/usr/bin/env python3
"""
Motor Test Script for Viam Rover 2 (Raspberry Pi 5)

Tests both motors by spinning them forward and backward.
Run this BEFORE setting up ROS2 to verify hardware works.

GPIO Pin Mapping (BCM):
  Left Motor:  PWM=26, A=19, B=13
  Right Motor: PWM=22, A=6,  B=5

Usage:
  python3 test_motors.py
  python3 test_motors.py --motor left
  python3 test_motors.py --motor right
  python3 test_motors.py --speed 50

Requirements:
  pip install lgpio  (for Pi 5)
"""

import argparse
import time
import sys

try:
    import lgpio
except ImportError:
    print("Error: lgpio not installed. Run: sudo apt install python3-lgpio")
    sys.exit(1)


# GPIO Pin definitions (BCM numbering)
MOTORS = {
    'left': {
        'pwm': 26,
        'a': 19,
        'b': 13,
    },
    'right': {
        'pwm': 22,
        'a': 6,
        'b': 5,
    }
}

PWM_FREQUENCY = 1000  # Hz


class Motor:
    def __init__(self, chip, name, pwm_pin, a_pin, b_pin):
        self.chip = chip
        self.name = name
        self.pwm_pin = pwm_pin
        self.a_pin = a_pin
        self.b_pin = b_pin

        # Set up GPIO pins
        lgpio.gpio_claim_output(chip, pwm_pin)
        lgpio.gpio_claim_output(chip, a_pin)
        lgpio.gpio_claim_output(chip, b_pin)

        print(f"  {name} motor: PWM={pwm_pin}, A={a_pin}, B={b_pin}")

    def set_speed(self, speed):
        """
        Set motor speed and direction.
        speed: -100 to 100 (negative = backward, positive = forward)
        """
        if speed > 0:
            # Forward: A=HIGH, B=LOW
            lgpio.gpio_write(self.chip, self.a_pin, 1)
            lgpio.gpio_write(self.chip, self.b_pin, 0)
        elif speed < 0:
            # Backward: A=LOW, B=HIGH
            lgpio.gpio_write(self.chip, self.a_pin, 0)
            lgpio.gpio_write(self.chip, self.b_pin, 1)
        else:
            # Stop: A=LOW, B=LOW
            lgpio.gpio_write(self.chip, self.a_pin, 0)
            lgpio.gpio_write(self.chip, self.b_pin, 0)

        # Set PWM duty cycle (0-100)
        duty = abs(speed)
        lgpio.tx_pwm(self.chip, self.pwm_pin, PWM_FREQUENCY, duty)

    def stop(self):
        """Stop the motor."""
        self.set_speed(0)
        lgpio.tx_pwm(self.chip, self.pwm_pin, PWM_FREQUENCY, 0)

    def brake(self):
        """Active brake: A=HIGH, B=HIGH."""
        lgpio.gpio_write(self.chip, self.a_pin, 1)
        lgpio.gpio_write(self.chip, self.b_pin, 1)
        lgpio.tx_pwm(self.chip, self.pwm_pin, PWM_FREQUENCY, 0)


def test_motor(motor, speed, duration):
    """Test a single motor forward and backward."""
    print(f"\n{'='*50}")
    print(f"Testing {motor.name} motor at {speed}% speed")
    print(f"{'='*50}")

    # Forward
    print(f"\n>>> {motor.name.upper()} FORWARD for {duration}s")
    print("    Watch the wheel - it should spin forward (robot moves forward)")
    motor.set_speed(speed)
    time.sleep(duration)
    motor.stop()
    time.sleep(0.5)

    # Backward
    print(f"\n>>> {motor.name.upper()} BACKWARD for {duration}s")
    print("    Watch the wheel - it should spin backward (robot moves backward)")
    motor.set_speed(-speed)
    time.sleep(duration)
    motor.stop()
    time.sleep(0.5)

    print(f"\n>>> {motor.name.upper()} STOPPED")


def main():
    parser = argparse.ArgumentParser(description='Test Rover2 motors')
    parser.add_argument('--motor', choices=['left', 'right', 'both'], default='both',
                        help='Which motor to test (default: both)')
    parser.add_argument('--speed', type=int, default=30,
                        help='Motor speed 0-100 (default: 30)')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Duration in seconds (default: 2.0)')
    args = parser.parse_args()

    # Clamp speed
    speed = max(0, min(100, args.speed))

    print("="*50)
    print("ROVER2 MOTOR TEST")
    print("="*50)
    print(f"\nSpeed: {speed}%")
    print(f"Duration: {args.duration}s per direction")
    print("\nInitializing GPIO...")

    # Open GPIO chip (Pi 5 uses gpiochip4)
    try:
        chip = lgpio.gpiochip_open(4)  # Pi 5
    except lgpio.error:
        try:
            chip = lgpio.gpiochip_open(0)  # Pi 4 fallback
        except lgpio.error as e:
            print(f"Error: Could not open GPIO chip: {e}")
            print("Make sure you're running on a Raspberry Pi with lgpio installed")
            sys.exit(1)

    print("GPIO initialized successfully")

    # Create motor objects
    motors = {}
    for name, pins in MOTORS.items():
        motors[name] = Motor(chip, name, pins['pwm'], pins['a'], pins['b'])

    print("\n" + "!"*50)
    print("WARNING: Motors will spin! Keep the rover wheels off the ground")
    print("         or place the rover on a safe surface.")
    print("!"*50)

    input("\nPress ENTER to start test (Ctrl+C to cancel)...")

    try:
        if args.motor == 'both':
            test_motor(motors['left'], speed, args.duration)
            test_motor(motors['right'], speed, args.duration)

            # Both together
            print(f"\n{'='*50}")
            print("Testing BOTH motors together")
            print(f"{'='*50}")

            print("\n>>> BOTH FORWARD")
            motors['left'].set_speed(speed)
            motors['right'].set_speed(speed)
            time.sleep(args.duration)

            print(">>> BOTH BACKWARD")
            motors['left'].set_speed(-speed)
            motors['right'].set_speed(-speed)
            time.sleep(args.duration)

            print(">>> SPIN LEFT (left back, right forward)")
            motors['left'].set_speed(-speed)
            motors['right'].set_speed(speed)
            time.sleep(args.duration)

            print(">>> SPIN RIGHT (left forward, right back)")
            motors['left'].set_speed(speed)
            motors['right'].set_speed(-speed)
            time.sleep(args.duration)

        else:
            test_motor(motors[args.motor], speed, args.duration)

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")

    finally:
        # Stop all motors
        print("\nStopping all motors...")
        for motor in motors.values():
            motor.stop()
        lgpio.gpiochip_close(chip)
        print("Done!")

    print("\n" + "="*50)
    print("TEST COMPLETE")
    print("="*50)
    print("""
Checklist:
  [ ] Left wheel spun forward when commanded forward
  [ ] Left wheel spun backward when commanded backward
  [ ] Right wheel spun forward when commanded forward
  [ ] Right wheel spun backward when commanded backward
  [ ] Both wheels spun same direction when both forward/backward
  [ ] Robot spun when wheels spun opposite directions

If a wheel spins the WRONG direction:
  → Swap the A and B pins in the config, OR
  → Swap the motor wires physically

If a wheel doesn't spin:
  → Check wiring connections
  → Check battery charge
  → Try higher speed (--speed 50)
""")


if __name__ == '__main__':
    main()
