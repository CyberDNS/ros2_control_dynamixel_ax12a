#ifndef DYNAMIXEL_HW__VELOCITY_CONTROLLER_HPP_
#define DYNAMIXEL_HW__VELOCITY_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>

namespace dynamixel_hw
{
  /**
   * PI correction on top of the driver's open-loop speed command.
   *
   * The AX-12A in wheel mode does not run at the commanded speed: measured on the robot it
   * falls short at low speed and overshoots at high speed, and the two servos differ by several
   * percent. This controller closes the loop on the measured wheel velocity:
   *
   *     output = target + kp * error + ki * integral(error)
   *
   * - The open-loop command is the feed-forward, so there is no step at the start of a motion.
   * - The integral is cleared when the target is zero or changes sign (no creeping at standstill).
   * - The integral is limited (anti-windup) and is frozen while the measurement is stale.
   * - Values below the servo's minimum speed are zeroed by the driver, so at very low speeds the
   *   integral makes the command alternate around that threshold, which averages to the target.
   */
  class VelocityController
  {
  public:
    struct Params
    {
      double kp = 0.3;              // [-]
      double ki = 2.0;              // [1/s]
      double max_correction = 3.0;  // largest |ki * integral| [rad/s]
      double max_output = 6.0;      // [rad/s]
    };

    VelocityController() = default;
    explicit VelocityController(const Params &params) : params_(params) {}

    void reset() { integral_ = 0.0; last_target_ = 0.0; }

    /**
     * @param target          requested wheel velocity [rad/s]
     * @param measured        measured wheel velocity [rad/s]
     * @param measured_valid  false while the measurement is stale (dead zone, no data yet)
     * @param dt              cycle time [s]
     * @return velocity command for the servo [rad/s]
     */
    double update(double target, double measured, bool measured_valid, double dt)
    {
      if (std::fabs(target) < 1e-3)
      {
        reset();
        return 0.0;
      }
      if (target * last_target_ < 0.0)
      {
        integral_ = 0.0; // direction change
      }
      last_target_ = target;

      double error = 0.0;
      if (measured_valid)
      {
        error = target - measured;
        const double limit = params_.max_correction / std::max(params_.ki, 1e-9);
        integral_ = std::clamp(integral_ + error * dt, -limit, limit);
      }

      const double output = target + params_.kp * error + params_.ki * integral_;
      return std::clamp(output, -params_.max_output, params_.max_output);
    }

    double integral() const { return integral_; }

  private:
    Params params_;
    double integral_ = 0.0;
    double last_target_ = 0.0;
  };
}

#endif // DYNAMIXEL_HW__VELOCITY_CONTROLLER_HPP_
