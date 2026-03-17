#include "rover2_base/gpio_encoder.hpp"

#include <cmath>
#include <iostream>

#ifdef HAS_LGPIO
#include <lgpio.h>
#endif

namespace rover2_base
{

// Global callback function for lgpio (needs C linkage)
#ifdef HAS_LGPIO
static void encoder_callback(int e, lgGpioAlert_p evt, void * userdata)
{
  (void)e;  // Unused
  (void)evt;  // We only care that an edge occurred
  auto encoder = static_cast<GPIOEncoder *>(userdata);
  if (encoder) {
    encoder->onPulse();
  }
}
#endif

GPIOEncoder::GPIOEncoder(const std::string & name, int pin, int ticks_per_rotation)
: name_(name),
  pin_(pin),
  ticks_per_rotation_(ticks_per_rotation),
  gpio_chip_(-1),
  tick_count_(0),
  direction_(1),
  last_tick_count_(0),
  initialized_(false),
  callback_id_(-1)
{
}

GPIOEncoder::~GPIOEncoder()
{
#ifdef HAS_LGPIO
  if (callback_id_ >= 0 && gpio_chip_ >= 0) {
    lgCallbackCancel(callback_id_);
  }
#endif
}

bool GPIOEncoder::init(int gpio_chip)
{
#ifdef HAS_LGPIO
  gpio_chip_ = gpio_chip;

  // Claim pin as input with pull-up
  int ret = lgGpioClaimInput(gpio_chip_, LG_SET_PULL_UP, pin_);
  if (ret < 0) {
    std::cerr << "[" << name_ << "] Failed to claim encoder pin " << pin_
              << ": " << lguErrorText(ret) << std::endl;
    return false;
  }

  // Set up edge detection callback
  callback_id_ = lgGpioSetSamplesFunc(encoder_callback, this);
  if (callback_id_ < 0) {
    // Try alternate method - alert on rising edge
    ret = lgGpioClaimAlert(gpio_chip_, 0, LG_RISING_EDGE, pin_, -1);
    if (ret < 0) {
      std::cerr << "[" << name_ << "] Failed to set up encoder callback: "
                << lguErrorText(ret) << std::endl;
      return false;
    }
  }

  initialized_ = true;
  std::cout << "[" << name_ << "] Encoder initialized: GPIO " << pin_
            << ", " << ticks_per_rotation_ << " ticks/rotation" << std::endl;
  return true;
#else
  (void)gpio_chip;
  std::cout << "[" << name_ << "] Encoder initialized (MOCK): GPIO " << pin_
            << ", " << ticks_per_rotation_ << " ticks/rotation" << std::endl;
  initialized_ = true;
  return true;
#endif
}

int64_t GPIOEncoder::getTicks() const
{
  return tick_count_.load();
}

double GPIOEncoder::getPosition() const
{
  double ticks = static_cast<double>(tick_count_.load());
  return (ticks / ticks_per_rotation_) * 2.0 * M_PI;
}

double GPIOEncoder::getVelocity(double dt)
{
  if (dt <= 0) return 0.0;

  int64_t current_ticks = tick_count_.load();
  int64_t delta_ticks = current_ticks - last_tick_count_;
  last_tick_count_ = current_ticks;

  double delta_rad = (static_cast<double>(delta_ticks) / ticks_per_rotation_) * 2.0 * M_PI;
  return delta_rad / dt;
}

void GPIOEncoder::setDirection(int direction)
{
  direction_.store(direction >= 0 ? 1 : -1);
}

void GPIOEncoder::reset()
{
  tick_count_.store(0);
  last_tick_count_ = 0;
}

void GPIOEncoder::onPulse()
{
  // Add tick in current direction
  tick_count_.fetch_add(direction_.load());
}

}  // namespace rover2_base
