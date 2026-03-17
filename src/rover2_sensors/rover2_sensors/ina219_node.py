#!/usr/bin/env python3
"""
INA219 Power Monitor ROS2 Node for Viam Rover 2

Publishes battery voltage, current, and power readings from INA219.

Topics:
  /battery_state (sensor_msgs/BatteryState) - Full battery status
  /battery/voltage (std_msgs/Float32) - Voltage only
  /battery/current (std_msgs/Float32) - Current only

Parameters:
  i2c_bus (int): I2C bus number (default: 1)
  i2c_address (int): I2C address (default: 0x40)
  publish_rate (float): Publishing rate in Hz (default: 10.0)
  shunt_resistance (float): Shunt resistor value in ohms (default: 0.1)
  low_voltage_threshold (float): Low battery warning threshold (default: 13.0)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

# Try to import smbus2, provide helpful error if not available
try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


# INA219 Register addresses
INA219_ADDR = 0x40
CONFIG_REG = 0x00
SHUNT_VOLTAGE_REG = 0x01
BUS_VOLTAGE_REG = 0x02
POWER_REG = 0x03
CURRENT_REG = 0x04
CALIBRATION_REG = 0x05

# INA219 Configuration: 32V range, 320mV shunt range, 12-bit, continuous
CONFIG_VALUE = 0x399F
CALIBRATION_VALUE = 4096
CURRENT_LSB = 0.1  # mA per bit


class INA219Driver:
    """Low-level driver for INA219 power monitor."""

    def __init__(self, bus_num=1, address=INA219_ADDR):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address

        # Configure the INA219
        self._write_register(CONFIG_REG, CONFIG_VALUE)
        self._write_register(CALIBRATION_REG, CALIBRATION_VALUE)

    def _write_register(self, register, value):
        """Write 16-bit value to register (big-endian)."""
        high = (value >> 8) & 0xFF
        low = value & 0xFF
        self.bus.write_i2c_block_data(self.address, register, [high, low])

    def _read_register(self, register):
        """Read 16-bit value from register (big-endian)."""
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        return (data[0] << 8) | data[1]

    def get_bus_voltage(self):
        """Get bus voltage in volts."""
        raw = self._read_register(BUS_VOLTAGE_REG)
        return (raw >> 3) * 0.004

    def get_current(self):
        """Get current in amps."""
        raw = self._read_register(CURRENT_REG)
        if raw > 32767:
            raw -= 65536
        return raw * CURRENT_LSB / 1000.0  # Convert mA to A

    def get_power(self):
        """Get power in watts."""
        raw = self._read_register(POWER_REG)
        return raw * 20 * CURRENT_LSB / 1000.0  # Convert mW to W

    def close(self):
        self.bus.close()


class INA219Node(Node):
    """ROS2 node for INA219 power monitor."""

    def __init__(self):
        super().__init__('ina219_node')

        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('shunt_resistance', 0.1)
        self.declare_parameter('low_voltage_threshold', 13.0)
        self.declare_parameter('critical_voltage_threshold', 12.0)

        # Get parameters
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.low_voltage_threshold = self.get_parameter('low_voltage_threshold').value
        self.critical_voltage_threshold = self.get_parameter('critical_voltage_threshold').value

        # Check if smbus2 is available
        if not SMBUS_AVAILABLE:
            self.get_logger().error(
                'smbus2 not installed! Run: sudo apt install python3-smbus2'
            )
            self.get_logger().warn('Publishing dummy data for testing')
            self.ina219 = None
        else:
            # Initialize INA219
            try:
                self.ina219 = INA219Driver(self.i2c_bus, self.i2c_address)
                self.get_logger().info(
                    f'INA219 initialized on bus {self.i2c_bus}, address 0x{self.i2c_address:02X}'
                )
            except Exception as e:
                self.get_logger().error(f'Failed to initialize INA219: {e}')
                self.get_logger().warn('Publishing dummy data for testing')
                self.ina219 = None

        # Create publishers
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.voltage_pub = self.create_publisher(Float32, 'battery/voltage', 10)
        self.current_pub = self.create_publisher(Float32, 'battery/current', 10)

        # Create timer
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(f'INA219 node started, publishing at {self.publish_rate} Hz')

    def timer_callback(self):
        """Read INA219 and publish data."""
        if self.ina219 is not None:
            try:
                voltage = self.ina219.get_bus_voltage()
                current = self.ina219.get_current()
                power = self.ina219.get_power()
            except Exception as e:
                self.get_logger().error(f'Error reading INA219: {e}')
                return
        else:
            # Dummy data for testing without hardware
            voltage = 14.8
            current = 0.5
            power = voltage * current

        # Create and publish BatteryState message
        battery_msg = BatteryState()
        battery_msg.header.stamp = self.get_clock().now().to_msg()
        battery_msg.header.frame_id = 'base_link'
        battery_msg.voltage = voltage
        battery_msg.current = current
        battery_msg.charge = float('nan')  # Unknown
        battery_msg.capacity = float('nan')  # Unknown
        battery_msg.design_capacity = float('nan')  # Unknown
        battery_msg.percentage = self._estimate_percentage(voltage)
        battery_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        battery_msg.power_supply_health = self._get_health(voltage)
        battery_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        battery_msg.present = True

        self.battery_pub.publish(battery_msg)

        # Publish individual topics
        self.voltage_pub.publish(Float32(data=voltage))
        self.current_pub.publish(Float32(data=current))

        # Log warnings for low battery
        if voltage < self.critical_voltage_threshold:
            self.get_logger().error(f'CRITICAL: Battery voltage {voltage:.2f}V!')
        elif voltage < self.low_voltage_threshold:
            self.get_logger().warn(f'Low battery: {voltage:.2f}V')

    def _estimate_percentage(self, voltage):
        """Estimate battery percentage from voltage (4S LiPo)."""
        # 4S LiPo: 12.0V (empty) to 16.8V (full)
        min_v = 12.0
        max_v = 16.8
        percentage = (voltage - min_v) / (max_v - min_v)
        return max(0.0, min(1.0, percentage))

    def _get_health(self, voltage):
        """Get battery health status."""
        if voltage < self.critical_voltage_threshold:
            return BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif voltage < self.low_voltage_threshold:
            return BatteryState.POWER_SUPPLY_HEALTH_COLD  # Using COLD as "low"
        else:
            return BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def destroy_node(self):
        """Clean up on shutdown."""
        if self.ina219 is not None:
            self.ina219.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = INA219Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
