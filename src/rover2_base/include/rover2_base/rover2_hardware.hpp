#ifndef ROVER2_BASE__ROVER2_HARDWARE_HPP_
#define ROVER2_BASE__ROVER2_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rover2_base/visibility_control.h"
#include "rover2_base/gpio_motor.hpp"
#include "rover2_base/gpio_encoder.hpp"

namespace rover2_base
{

/**
 * @brief ros2_control hardware interface for Viam Rover 2
 *
 * This hardware interface controls:
 * - Two DC motors via GPIO (PWM + direction pins)
 * - Two single-phase encoders via GPIO
 *
 * It provides velocity command interfaces and position/velocity state interfaces
 * for use with the diff_drive_controller.
 *
 * GPIO Configuration (BCM numbering):
 *   Left Motor:  PWM=26, A=19, B=13
 *   Right Motor: PWM=22, A=6, B=5
 *   Left Encoder:  GPIO 20
 *   Right Encoder: GPIO 21
 */
class Rover2Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Rover2Hardware)

  ROVER2_BASE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROVER2_BASE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVER2_BASE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVER2_BASE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVER2_BASE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ROVER2_BASE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROVER2_BASE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROVER2_BASE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROVER2_BASE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // GPIO configuration
  struct MotorConfig {
    int pwm_pin;
    int a_pin;
    int b_pin;
  };

  struct EncoderConfig {
    int pin;
    int ticks_per_rotation;
  };

  // Hardware parameters
  MotorConfig left_motor_config_;
  MotorConfig right_motor_config_;
  EncoderConfig left_encoder_config_;
  EncoderConfig right_encoder_config_;
  double max_velocity_;  // rad/s

  // Hardware objects
  std::unique_ptr<GPIOMotor> left_motor_;
  std::unique_ptr<GPIOMotor> right_motor_;
  std::unique_ptr<GPIOEncoder> left_encoder_;
  std::unique_ptr<GPIOEncoder> right_encoder_;

  // GPIO chip handle
  int gpio_chip_;

  // State interfaces (position and velocity for each wheel)
  double hw_positions_[2];
  double hw_velocities_[2];

  // Command interfaces (velocity for each wheel)
  double hw_commands_[2];

  // Joint names
  std::vector<std::string> joint_names_;

  // Per-wheel PI controller state
  double pid_integral_[2] = {0.0, 0.0};
  double pid_output_[2] = {0.0, 0.0};
  static constexpr double PID_KP = 0.5;    // Proportional gain (duty % per rad/s error)
  static constexpr double PID_KI = 0.1;    // Integral gain
  static constexpr double PID_MAX_I = 10.0; // Anti-windup limit on integral term

  // Helper to parse parameters
  bool parseParameters(const hardware_interface::HardwareInfo & info);
};

}  // namespace rover2_base

#endif  // ROVER2_BASE__ROVER2_HARDWARE_HPP_
