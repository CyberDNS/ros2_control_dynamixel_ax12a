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
      double kpos = 4.0;            // position-tracking gain [1/s]; 0 disables
      double max_pos_correction = 0.8;  // largest |kpos * position error| [rad/s]
    };

    VelocityController() = default;
    explicit VelocityController(const Params &params) : params_(params) {}

    void reset() { integral_ = 0.0; last_target_ = 0.0; have_ref_ = false; }

    /**
     * @param target          requested wheel velocity [rad/s]
     * @param measured        measured wheel velocity [rad/s]
     * @param measured_valid  false while the measurement is stale (dead zone, no data yet)
     * @param dt              cycle time [s]
     * @return velocity command for the servo [rad/s]
     */
    double update(double target, double measured, bool measured_valid, double dt)
    {
      return update(target, measured, measured_valid, dt, 0.0, false);
    }

    /**
     * Same, with position tracking: the wheel is also pulled towards a reference position that
     * integrates the target velocity from the start of the motion. A servo that starts late or
     * runs slow during the acceleration then catches up, so the two wheels of the robot stay in
     * step (otherwise a start-up lag of one wheel stays as a heading error).
     *
     * @param position      measured wheel position [rad], only used if @p position_valid
     */
    double update(double target, double measured, bool measured_valid, double dt,
                  double position, bool position_valid)
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

      if (position_valid && params_.kpos > 0.0)
      {
        if (!have_ref_ || target * ref_dir_ < 0.0)
        {
          pos_ref_ = position;
          have_ref_ = true;
          ref_dir_ = target;
        }
        pos_ref_ += target * dt;
        // a blocked wheel must not build up an unbounded reference
        const double limit = params_.max_pos_correction / params_.kpos;
        pos_ref_ = std::clamp(pos_ref_, position - limit, position + limit);
        target += std::clamp(params_.kpos * (pos_ref_ - position), -params_.max_pos_correction,
                             params_.max_pos_correction);
      }

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
    double pos_ref_ = 0.0;
    double ref_dir_ = 0.0;
    bool have_ref_ = false;
  };
}

#endif // DYNAMIXEL_HW__VELOCITY_CONTROLLER_HPP_
