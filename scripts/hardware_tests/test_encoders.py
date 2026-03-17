#!/usr/bin/env python3
"""
Encoder Test Script for Viam Rover 2 (Raspberry Pi 5)

Tests wheel encoders by counting pulses as you manually rotate the wheels.
Run this BEFORE setting up ROS2 to verify hardware works.

GPIO Pin Mapping (BCM):
  Left Encoder:  GPIO 20 (Physical pin 38)
  Right Encoder: GPIO 21 (Physical pin 40)

Expected: ~1992 ticks per full wheel rotation (after gearbox)

Usage:
  python3 test_encoders.py
  python3 test_encoders.py --duration 30

Requirements:
  pip install lgpio  (for Pi 5)
"""

import argparse
import time
import sys
import threading

try:
    import lgpio
except ImportError:
    print("Error: lgpio not installed. Run: sudo apt install python3-lgpio")
    sys.exit(1)


# GPIO Pin definitions (BCM numbering)
ENCODERS = {
    'left': 20,   # Physical pin 38
    'right': 21,  # Physical pin 40
}

TICKS_PER_ROTATION = 1992  # Expected ticks per full wheel rotation


class EncoderCounter:
    def __init__(self, chip, name, pin):
        self.chip = chip
        self.name = name
        self.pin = pin
        self.count = 0
        self.last_time = time.time()
        self.frequency = 0.0
        self._lock = threading.Lock()

        # Set up GPIO pin as input with pull-up
        lgpio.gpio_claim_input(chip, pin, lgpio.SET_PULL_UP)

        # Set up edge detection callback
        self._cb = lgpio.callback(chip, pin, lgpio.RISING_EDGE, self._callback)

        print(f"  {name} encoder: GPIO {pin}")

    def _callback(self, chip, gpio, level, tick):
        """Called on each rising edge (encoder pulse)."""
        with self._lock:
            self.count += 1
            now = time.time()
            dt = now - self.last_time
            if dt > 0:
                self.frequency = 1.0 / dt
            self.last_time = now

    def get_count(self):
        """Get current tick count."""
        with self._lock:
            return self.count

    def get_rotations(self):
        """Get number of full wheel rotations."""
        return self.get_count() / TICKS_PER_ROTATION

    def reset(self):
        """Reset the counter."""
        with self._lock:
            self.count = 0

    def stop(self):
        """Stop the callback."""
        self._cb.cancel()


def main():
    parser = argparse.ArgumentParser(description='Test Rover2 wheel encoders')
    parser.add_argument('--duration', type=int, default=60,
                        help='Test duration in seconds (default: 60)')
    args = parser.parse_args()

    print("="*60)
    print("ROVER2 ENCODER TEST")
    print("="*60)
    print(f"\nExpected: {TICKS_PER_ROTATION} ticks per full wheel rotation")
    print("\nInitializing GPIO...")

    # Open GPIO chip (Pi 5 uses gpiochip4)
    try:
        chip = lgpio.gpiochip_open(4)  # Pi 5
    except lgpio.error:
        try:
            chip = lgpio.gpiochip_open(0)  # Pi 4 fallback
        except lgpio.error as e:
            print(f"Error: Could not open GPIO chip: {e}")
            sys.exit(1)

    print("GPIO initialized successfully")

    # Create encoder counters
    encoders = {}
    for name, pin in ENCODERS.items():
        encoders[name] = EncoderCounter(chip, name, pin)

    print("\n" + "="*60)
    print("INSTRUCTIONS:")
    print("  1. Lift the rover so wheels can spin freely")
    print("  2. Slowly rotate each wheel BY HAND")
    print("  3. Watch the tick count increase")
    print("  4. One full rotation should give ~1992 ticks")
    print("="*60)

    input("\nPress ENTER to start counting (Ctrl+C to stop)...")

    print("\n" + "-"*60)
    print(f"{'Wheel':<10} {'Ticks':>10} {'Rotations':>12} {'Ticks/sec':>12}")
    print("-"*60)

    try:
        start_time = time.time()
        last_counts = {'left': 0, 'right': 0}
        last_time = start_time

        while time.time() - start_time < args.duration:
            time.sleep(0.2)  # Update display every 200ms

            now = time.time()
            dt = now - last_time

            for name, encoder in encoders.items():
                count = encoder.get_count()
                rotations = encoder.get_rotations()

                # Calculate ticks per second
                ticks_delta = count - last_counts[name]
                ticks_per_sec = ticks_delta / dt if dt > 0 else 0
                last_counts[name] = count

                print(f"\r{name:<10} {count:>10} {rotations:>12.3f} {ticks_per_sec:>12.1f}", end="")

            last_time = now

            # Move cursor up for next encoder
            print("\033[F" * len(encoders), end="")

        # Final newlines
        print("\n" * len(encoders))

    except KeyboardInterrupt:
        print("\n\nTest stopped by user")

    finally:
        # Print final results
        print("\n" + "="*60)
        print("FINAL RESULTS")
        print("="*60)

        for name, encoder in encoders.items():
            count = encoder.get_count()
            rotations = encoder.get_rotations()
            print(f"\n{name.upper()} ENCODER:")
            print(f"  Total ticks: {count}")
            print(f"  Rotations:   {rotations:.3f}")

            if count > 0:
                ticks_per_rot = count / rotations if rotations > 0 else 0
                if abs(ticks_per_rot - TICKS_PER_ROTATION) < 100:
                    print(f"  Status:      OK (matches expected ~{TICKS_PER_ROTATION})")
                else:
                    print(f"  Status:      WARNING - Expected ~{TICKS_PER_ROTATION} ticks/rotation")

            encoder.stop()

        lgpio.gpiochip_close(chip)

    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print("""
Checklist:
  [ ] Left encoder counts increase when left wheel rotates
  [ ] Right encoder counts increase when right wheel rotates
  [ ] ~1992 ticks per full rotation (within ~5%)

If encoder doesn't count:
  → Check wiring connections
  → Verify correct GPIO pin
  → Check encoder power supply (usually 3.3V or 5V)

If count is wrong:
  → Note the actual ticks per rotation
  → Update TICKS_PER_ROTATION in rover2 config
""")


if __name__ == '__main__':
    main()
