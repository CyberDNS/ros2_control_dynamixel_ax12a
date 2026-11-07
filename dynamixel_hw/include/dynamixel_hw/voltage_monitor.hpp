#ifndef DYNAMIXEL_HW__VOLTAGE_MONITOR_HPP_
#define DYNAMIXEL_HW__VOLTAGE_MONITOR_HPP_

namespace dynamixel_hw
{
  /**
   * Classifies the servo supply voltage. The AX-12A reports it in register 42 (0.1 V units).
   *
   * - kWarn: below `warn_volts` (battery getting low, or the supply sags under load)
   * - kStop: below `stop_volts` for `stop_after` consecutive readings (debounce against glitches)
   * A threshold of 0 disables that check. Implausible readings (<= 0 or > 30 V, i.e. a failed
   * or corrupted read) are ignored.
   */
  class VoltageMonitor
  {
  public:
    enum class State { kOk, kWarn, kStop };

    struct Params
    {
      double warn_volts = 10.5; // 3.5 V per cell on a 3S pack
      double stop_volts = 9.6;  // 3.2 V per cell; AX-12A minimum is 9 V
      int stop_after = 3;
    };

    VoltageMonitor() = default;
    explicit VoltageMonitor(const Params &params) : params_(params) {}

    State update(double volts)
    {
      if (volts <= 0.0 || volts > 30.0)
      {
        return State::kOk;
      }
      if (params_.stop_volts > 0.0 && volts < params_.stop_volts)
      {
        ++below_stop_;
        return below_stop_ >= params_.stop_after ? State::kStop : State::kWarn;
      }
      below_stop_ = 0;
      if (params_.warn_volts > 0.0 && volts < params_.warn_volts)
      {
        return State::kWarn;
      }
      return State::kOk;
    }

  private:
    Params params_;
    int below_stop_ = 0;
  };
}

#endif // DYNAMIXEL_HW__VOLTAGE_MONITOR_HPP_
