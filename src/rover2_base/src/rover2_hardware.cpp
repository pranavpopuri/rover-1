#include "rover2_base/rover2_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef HAS_LGPIO
#include <lgpio.h>
#endif

namespace rover2_base
{

hardware_interface::CallbackReturn Rover2Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse hardware parameters from URDF
  if (!parseParameters(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize state/command arrays
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;
  hw_velocities_[0] = 0.0;
  hw_velocities_[1] = 0.0;
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  // Store joint names
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);
  }

  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"),
              "Hardware interface initialized for %zu joints", info.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool Rover2Hardware::parseParameters(const hardware_interface::HardwareInfo & info)
{
  // Default values (from Viam Rover 2 spec)
  left_motor_config_ = {26, 19, 13};   // PWM, A, B
  right_motor_config_ = {22, 6, 5};    // PWM, A, B
  left_encoder_config_ = {20, 1992};   // Pin, ticks per rotation
  right_encoder_config_ = {21, 1992};  // Pin, ticks per rotation
  max_velocity_ = 7.54;  // rad/s (~120 RPM)

  // Parse from hardware parameters if provided
  auto it = info.hardware_parameters.find("left_motor_pwm_pin");
  if (it != info.hardware_parameters.end()) {
    left_motor_config_.pwm_pin = std::stoi(it->second);
  }
  it = info.hardware_parameters.find("left_motor_a_pin");
  if (it != info.hardware_parameters.end()) {
    left_motor_config_.a_pin = std::stoi(it->second);
  }
  it = info.hardware_parameters.find("left_motor_b_pin");
  if (it != info.hardware_parameters.end()) {
    left_motor_config_.b_pin = std::stoi(it->second);
  }

  it = info.hardware_parameters.find("right_motor_pwm_pin");
  if (it != info.hardware_parameters.end()) {
    right_motor_config_.pwm_pin = std::stoi(it->second);
  }
  it = info.hardware_parameters.find("right_motor_a_pin");
  if (it != info.hardware_parameters.end()) {
    right_motor_config_.a_pin = std::stoi(it->second);
  }
  it = info.hardware_parameters.find("right_motor_b_pin");
  if (it != info.hardware_parameters.end()) {
    right_motor_config_.b_pin = std::stoi(it->second);
  }

  it = info.hardware_parameters.find("left_encoder_pin");
  if (it != info.hardware_parameters.end()) {
    left_encoder_config_.pin = std::stoi(it->second);
  }
  it = info.hardware_parameters.find("right_encoder_pin");
  if (it != info.hardware_parameters.end()) {
    right_encoder_config_.pin = std::stoi(it->second);
  }

  it = info.hardware_parameters.find("encoder_ticks_per_rotation");
  if (it != info.hardware_parameters.end()) {
    int ticks = std::stoi(it->second);
    left_encoder_config_.ticks_per_rotation = ticks;
    right_encoder_config_.ticks_per_rotation = ticks;
  }

  it = info.hardware_parameters.find("max_velocity");
  if (it != info.hardware_parameters.end()) {
    max_velocity_ = std::stod(it->second);
  }

  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"),
              "Left motor: PWM=%d, A=%d, B=%d",
              left_motor_config_.pwm_pin,
              left_motor_config_.a_pin,
              left_motor_config_.b_pin);
  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"),
              "Right motor: PWM=%d, A=%d, B=%d",
              right_motor_config_.pwm_pin,
              right_motor_config_.a_pin,
              right_motor_config_.b_pin);
  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"),
              "Encoders: Left=%d, Right=%d, Ticks=%d",
              left_encoder_config_.pin,
              right_encoder_config_.pin,
              left_encoder_config_.ticks_per_rotation);

  return true;
}

hardware_interface::CallbackReturn Rover2Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
#ifdef HAS_LGPIO
  // Open GPIO chip (Pi 5 uses gpiochip4, Pi 4 uses gpiochip0)
  gpio_chip_ = lgGpiochipOpen(4);
  if (gpio_chip_ < 0) {
    gpio_chip_ = lgGpiochipOpen(0);
  }
  if (gpio_chip_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2Hardware"),
                 "Failed to open GPIO chip: %s", lguErrorText(gpio_chip_));
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"),
              "GPIO chip opened: %d", gpio_chip_);
#else
  gpio_chip_ = 0;
  RCLCPP_WARN(rclcpp::get_logger("Rover2Hardware"),
              "Running without lgpio - MOCK mode");
#endif

  // Create motor objects
  left_motor_ = std::make_unique<GPIOMotor>(
    "left_motor",
    left_motor_config_.pwm_pin,
    left_motor_config_.a_pin,
    left_motor_config_.b_pin);

  right_motor_ = std::make_unique<GPIOMotor>(
    "right_motor",
    right_motor_config_.pwm_pin,
    right_motor_config_.a_pin,
    right_motor_config_.b_pin);

  // Create encoder objects
  left_encoder_ = std::make_unique<GPIOEncoder>(
    "left_encoder",
    left_encoder_config_.pin,
    left_encoder_config_.ticks_per_rotation);

  right_encoder_ = std::make_unique<GPIOEncoder>(
    "right_encoder",
    right_encoder_config_.pin,
    right_encoder_config_.ticks_per_rotation);

  // Initialize all hardware
  if (!left_motor_->init(gpio_chip_) || !right_motor_->init(gpio_chip_) ||
      !left_encoder_->init(gpio_chip_) || !right_encoder_->init(gpio_chip_))
  {
    RCLCPP_ERROR(rclcpp::get_logger("Rover2Hardware"),
                 "Failed to initialize hardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"), "Hardware configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rover2Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset encoders
  left_encoder_->reset();
  right_encoder_->reset();

  // Reset command values
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rover2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop motors and encoder polling threads
  left_motor_->stop();
  right_motor_->stop();
  left_encoder_->stop();
  right_encoder_->stop();

  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Rover2Hardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop motors
  if (left_motor_) left_motor_->stop();
  if (right_motor_) right_motor_->stop();

#ifdef HAS_LGPIO
  if (gpio_chip_ >= 0) {
    lgGpiochipClose(gpio_chip_);
    gpio_chip_ = -1;
  }
#endif

  RCLCPP_INFO(rclcpp::get_logger("Rover2Hardware"), "Hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Rover2Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_names_[0], hardware_interface::HW_IF_POSITION, &hw_positions_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_names_[0], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]));

  // Right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_names_[1], hardware_interface::HW_IF_POSITION, &hw_positions_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    joint_names_[1], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Rover2Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Velocity command for left wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_names_[0], hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]));

  // Velocity command for right wheel
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    joint_names_[1], hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]));

  return command_interfaces;
}

hardware_interface::return_type Rover2Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  // Read encoder positions (polling thread counts ticks in background)
  hw_positions_[0] = left_encoder_->getPosition();
  hw_positions_[1] = right_encoder_->getPosition();

  // Read encoder velocities
  hw_velocities_[0] = left_encoder_->getVelocity(dt);
  hw_velocities_[1] = right_encoder_->getVelocity(dt);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Rover2Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Set encoder directions based on commanded velocity
  left_encoder_->setDirection(hw_commands_[0] >= 0 ? 1 : -1);
  right_encoder_->setDirection(hw_commands_[1] >= 0 ? 1 : -1);

  // Send velocity commands to motors (open-loop)
  left_motor_->setVelocity(hw_commands_[0], max_velocity_);
  right_motor_->setVelocity(hw_commands_[1], max_velocity_);

  return hardware_interface::return_type::OK;
}

}  // namespace rover2_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rover2_base::Rover2Hardware, hardware_interface::SystemInterface)
