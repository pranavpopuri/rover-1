#ifndef ROVER2_BASE__GPIO_MOTOR_HPP_
#define ROVER2_BASE__GPIO_MOTOR_HPP_

#include <string>

namespace rover2_base
{

/**
 * @brief GPIO-based motor driver for H-Bridge (PWM + A/B direction pins)
 *
 * Controls a DC motor using:
 * - PWM pin for speed control (0-100% duty cycle)
 * - A pin for forward direction
 * - B pin for backward direction
 *
 * Direction logic:
 *   A=HIGH, B=LOW  → Forward
 *   A=LOW,  B=HIGH → Backward
 *   A=LOW,  B=LOW  → Coast (stop)
 *   A=HIGH, B=HIGH → Brake
 */
class GPIOMotor
{
public:
  GPIOMotor(const std::string & name, int pwm_pin, int a_pin, int b_pin);
  ~GPIOMotor();

  /**
   * @brief Initialize GPIO pins
   * @param gpio_chip GPIO chip handle (from lgpio)
   * @return true if successful
   */
  bool init(int gpio_chip);

  /**
   * @brief Set motor velocity
   * @param velocity Velocity in rad/s (positive=forward, negative=backward)
   * @param max_velocity Maximum velocity in rad/s (for scaling to PWM)
   */
  void setVelocity(double velocity, double max_velocity);

  /**
   * @brief Stop the motor (coast)
   */
  void stop();

  /**
   * @brief Brake the motor (active stop)
   */
  void brake();

  /**
   * @brief Get the last commanded direction
   * @return 1 for forward, -1 for backward, 0 for stopped
   */
  int getDirection() const { return direction_; }

private:
  std::string name_;
  int pwm_pin_;
  int a_pin_;
  int b_pin_;
  int gpio_chip_;
  int direction_;
  bool initialized_;

  static constexpr int PWM_FREQUENCY = 1000;  // Hz
};

}  // namespace rover2_base

#endif  // ROVER2_BASE__GPIO_MOTOR_HPP_
