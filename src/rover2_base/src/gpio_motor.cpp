#include "rover2_base/gpio_motor.hpp"

#include <cmath>
#include <iostream>

#ifdef HAS_LGPIO
#include <lgpio.h>
#endif

namespace rover2_base
{

GPIOMotor::GPIOMotor(const std::string & name, int pwm_pin, int a_pin, int b_pin)
: name_(name),
  pwm_pin_(pwm_pin),
  a_pin_(a_pin),
  b_pin_(b_pin),
  gpio_chip_(-1),
  direction_(0),
  initialized_(false)
{
}

GPIOMotor::~GPIOMotor()
{
  if (initialized_) {
    stop();
  }
}

bool GPIOMotor::init(int gpio_chip)
{
#ifdef HAS_LGPIO
  gpio_chip_ = gpio_chip;

  // Claim PWM pin as output
  int ret = lgGpioClaimOutput(gpio_chip_, 0, pwm_pin_, 0);
  if (ret < 0) {
    std::cerr << "[" << name_ << "] Failed to claim PWM pin " << pwm_pin_
              << ": " << lguErrorText(ret) << std::endl;
    return false;
  }

  // Claim direction pins as outputs
  ret = lgGpioClaimOutput(gpio_chip_, 0, a_pin_, 0);
  if (ret < 0) {
    std::cerr << "[" << name_ << "] Failed to claim A pin " << a_pin_
              << ": " << lguErrorText(ret) << std::endl;
    return false;
  }

  ret = lgGpioClaimOutput(gpio_chip_, 0, b_pin_, 0);
  if (ret < 0) {
    std::cerr << "[" << name_ << "] Failed to claim B pin " << b_pin_
              << ": " << lguErrorText(ret) << std::endl;
    return false;
  }

  initialized_ = true;
  std::cout << "[" << name_ << "] Motor initialized: PWM=" << pwm_pin_
            << ", A=" << a_pin_ << ", B=" << b_pin_ << std::endl;
  return true;
#else
  (void)gpio_chip;
  std::cout << "[" << name_ << "] Motor initialized (MOCK): PWM=" << pwm_pin_
            << ", A=" << a_pin_ << ", B=" << b_pin_ << std::endl;
  initialized_ = true;
  return true;
#endif
}

void GPIOMotor::setVelocity(double velocity, double max_velocity)
{
  if (!initialized_) return;

  // Calculate duty cycle (0-100)
  double duty = std::abs(velocity) / max_velocity * 100.0;
  duty = std::min(100.0, std::max(0.0, duty));

  // Determine direction
  int new_direction = 0;
  if (velocity > 0.01) {
    new_direction = 1;  // Forward
  } else if (velocity < -0.01) {
    new_direction = -1;  // Backward
  }

#ifdef HAS_LGPIO
  // Set direction pins
  if (new_direction > 0) {
    lgGpioWrite(gpio_chip_, a_pin_, 1);
    lgGpioWrite(gpio_chip_, b_pin_, 0);
  } else if (new_direction < 0) {
    lgGpioWrite(gpio_chip_, a_pin_, 0);
    lgGpioWrite(gpio_chip_, b_pin_, 1);
  } else {
    lgGpioWrite(gpio_chip_, a_pin_, 0);
    lgGpioWrite(gpio_chip_, b_pin_, 0);
  }

  // Set PWM
  lgTxPwm(gpio_chip_, pwm_pin_, PWM_FREQUENCY, duty, 0, 0);
#endif

  direction_ = new_direction;
}

void GPIOMotor::stop()
{
  if (!initialized_) return;

#ifdef HAS_LGPIO
  lgGpioWrite(gpio_chip_, a_pin_, 0);
  lgGpioWrite(gpio_chip_, b_pin_, 0);
  lgTxPwm(gpio_chip_, pwm_pin_, PWM_FREQUENCY, 0, 0, 0);
#endif

  direction_ = 0;
}

void GPIOMotor::brake()
{
  if (!initialized_) return;

#ifdef HAS_LGPIO
  lgGpioWrite(gpio_chip_, a_pin_, 1);
  lgGpioWrite(gpio_chip_, b_pin_, 1);
  lgTxPwm(gpio_chip_, pwm_pin_, PWM_FREQUENCY, 0, 0, 0);
#endif

  direction_ = 0;
}

}  // namespace rover2_base
