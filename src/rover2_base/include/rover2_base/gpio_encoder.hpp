#ifndef ROVER2_BASE__GPIO_ENCODER_HPP_
#define ROVER2_BASE__GPIO_ENCODER_HPP_

#include <atomic>
#include <string>

namespace rover2_base
{

/**
 * @brief GPIO-based single-phase encoder reader
 *
 * Counts rising edges on encoder pin.
 * Since this is a single-phase encoder (not quadrature),
 * direction must be inferred from motor commands.
 *
 * The encoder provides ~1992 ticks per wheel rotation.
 */
class GPIOEncoder
{
public:
  GPIOEncoder(const std::string & name, int pin, int ticks_per_rotation);
  ~GPIOEncoder();

  /**
   * @brief Initialize GPIO pin for edge detection
   * @param gpio_chip GPIO chip handle (from lgpio)
   * @return true if successful
   */
  bool init(int gpio_chip);

  /**
   * @brief Get current tick count (signed, based on direction)
   */
  int64_t getTicks() const;

  /**
   * @brief Get wheel position in radians
   */
  double getPosition() const;

  /**
   * @brief Get wheel velocity in rad/s
   * @param dt Time since last call in seconds
   */
  double getVelocity(double dt);

  /**
   * @brief Set the counting direction (based on motor command)
   * @param direction 1 for forward, -1 for backward
   */
  void setDirection(int direction);

  /**
   * @brief Reset tick count to zero
   */
  void reset();

  /**
   * @brief Called by lgpio callback on rising edge
   */
  void onPulse();

private:
  std::string name_;
  int pin_;
  int ticks_per_rotation_;
  int gpio_chip_;
  std::atomic<int64_t> tick_count_;
  std::atomic<int> direction_;
  int64_t last_tick_count_;
  bool initialized_;

  // Callback ID for lgpio
  int callback_id_;
};

}  // namespace rover2_base

#endif  // ROVER2_BASE__GPIO_ENCODER_HPP_
