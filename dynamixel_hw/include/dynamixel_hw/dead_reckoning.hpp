#ifndef DYNAMIXEL_HW__DEAD_RECKONING_HPP_
#define DYNAMIXEL_HW__DEAD_RECKONING_HPP_

#include <cmath>

namespace dynamixel_hw
{
  /**
   * Velocity used to predict the wheel motion while the potentiometer cannot be read
   * (dead zone, including the 1023 / 0 plateaus).
   *
   * The measured velocity predicts better than the command (the servo does not run at the
   * commanded speed), but it is frozen while readings are rejected. A wheel that has been told
   * to stop and rests inside the dead zone would then be dead-reckoned forever at its old speed.
   * So a zero command always predicts no motion.
   *
   * @param commanded  velocity the servo was last told to run at [rad/s]
   * @param measured   last good measured velocity [rad/s]
   */
  inline double prediction_velocity(double commanded, double measured)
  {
    if (std::fabs(commanded) < 1e-6)
    {
      return 0.0;
    }
    // use the measurement only if it agrees in direction and is not negligible
    if (std::fabs(measured) > 0.1 && measured * commanded > 0.0)
    {
      return measured;
    }
    return commanded;
  }
}

#endif // DYNAMIXEL_HW__DEAD_RECKONING_HPP_
