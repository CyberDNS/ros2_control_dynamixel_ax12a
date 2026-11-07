#ifndef DYNAMIXEL_HW__VELOCITY_ESTIMATOR_HPP_
#define DYNAMIXEL_HW__VELOCITY_ESTIMATOR_HPP_

#include <array>
#include <cstddef>

namespace dynamixel_hw
{
  /**
   * Wheel velocity from position samples, as the average over a short window.
   *
   * One potentiometer count is ~0.005 rad, which is ~0.15 rad/s noise for a single 30 Hz
   * difference. Averaging over `kWindow` cycles reduces that to ~0.04 rad/s at the cost of
   * ~2 cycles of delay.
   *
   * A sample can be flagged "unclean" (dead-reckoned inside the potentiometer dead zone, or the
   * first reading after it, where the position is re-synchronised with a jump). Windows that
   * contain such a sample would give a wrong velocity, so the estimate is frozen for `kWindow`
   * cycles after the last unclean sample and `valid()` is false meanwhile.
   */
  class VelocityEstimator
  {
  public:
    static constexpr std::size_t kWindow = 4;

    void reset(double position)
    {
      positions_.fill(position);
      times_.fill(0.0);
      count_ = 0;
      hold_ = kWindow;
      velocity_ = 0.0;
      valid_ = false;
    }

    /// @param position wheel position [rad]; @param dt time since the previous sample [s]
    void push(double position, double dt, bool clean)
    {
      // shift history
      for (std::size_t i = 0; i < kWindow; ++i)
      {
        positions_[i] = positions_[i + 1];
        times_[i] = times_[i + 1];
      }
      positions_[kWindow] = position;
      times_[kWindow] = dt;
      if (count_ < kWindow)
      {
        ++count_;
      }

      if (!clean)
      {
        hold_ = kWindow;
      }
      else if (hold_ > 0)
      {
        --hold_;
      }

      valid_ = (hold_ == 0) && (count_ >= kWindow);
      if (valid_)
      {
        double span = 0.0;
        for (std::size_t i = 1; i <= kWindow; ++i)
        {
          span += times_[i];
        }
        if (span > 0.0)
        {
          velocity_ = (positions_[kWindow] - positions_[0]) / span;
        }
        else
        {
          valid_ = false;
        }
      }
    }

    /// Last good estimate [rad/s] (held while !valid()).
    double velocity() const { return velocity_; }
    /// True when the current estimate is fresh.
    bool valid() const { return valid_; }

  private:
    std::array<double, kWindow + 1> positions_{};
    std::array<double, kWindow + 1> times_{};
    std::size_t count_ = 0;
    std::size_t hold_ = kWindow;
    double velocity_ = 0.0;
    bool valid_ = false;
  };
}

#endif // DYNAMIXEL_HW__VELOCITY_ESTIMATOR_HPP_
