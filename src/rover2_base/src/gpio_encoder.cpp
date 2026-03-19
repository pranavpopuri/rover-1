#include "rover2_base/gpio_encoder.hpp"

#include <chrono>
#include <cmath>
#include <iostream>

#ifdef HAS_LGPIO
#include <lgpio.h>
#endif

namespace rover2_base
{

GPIOEncoder::GPIOEncoder(const std::string & name, int pin, int ticks_per_rotation)
: name_(name),
  pin_(pin),
  ticks_per_rotation_(ticks_per_rotation),
  gpio_chip_(-1),
  tick_count_(0),
  direction_(1),
  last_tick_count_(0),
  initialized_(false),
  running_(false)
{
}

GPIOEncoder::~GPIOEncoder()
{
  stop();
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

  // Start polling thread
  running_ = true;
  poll_thread_ = std::thread(&GPIOEncoder::pollLoop, this);

  initialized_ = true;
  std::cout << "[" << name_ << "] Encoder initialized (polling thread): GPIO " << pin_
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

void GPIOEncoder::stop()
{
  running_ = false;
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
}

void GPIOEncoder::pollLoop()
{
#ifdef HAS_LGPIO
  int last_state = lgGpioRead(gpio_chip_, pin_);

  while (running_) {
    int state = lgGpioRead(gpio_chip_, pin_);
    if (state >= 0 && state == 1 && last_state == 0) {
      // Rising edge detected
      tick_count_.fetch_add(direction_.load());
    }
    if (state >= 0) {
      last_state = state;
    }
    // Poll at ~20kHz — fast enough to catch all ticks at max motor speed
    // At 120 RPM with 1992 ticks/rotation: ~3984 edges/sec
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
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

}  // namespace rover2_base
