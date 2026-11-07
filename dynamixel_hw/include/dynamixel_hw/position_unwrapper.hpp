#ifndef DYNAMIXEL_HW__POSITION_UNWRAPPER_HPP_
#define DYNAMIXEL_HW__POSITION_UNWRAPPER_HPP_

#include <cmath>

namespace dynamixel_hw
{
  /**
   * Turns the raw AX-12A potentiometer reading into a continuous (unwrapped) position.
   *
   * The AX-12A reports its position as 0..1023 over 300 degrees. The remaining 60 degrees
   * of a revolution are a dead zone where the potentiometer wiper is not on the track and
   * the reading is garbage (measured on the robot: values like 630..650 between 1023 and 0).
   *
   * Strategy: every reading is turned into the revolution-unwrapped candidate that is closest
   * to a prediction (last position + expected motion). If that candidate is within
   * `tolerance_counts` of the prediction it is accepted; otherwise the reading is rejected
   * (dead zone or glitch) and the prediction is used instead. The dead zone has a fixed width,
   * so the first valid reading after it re-synchronises the position without accumulated error.
   * Optionally (`resync_after` > 0) the nearest candidate is accepted after that many rejections
   * in a row. This is off by default: a wheel resting in the dead zone reads the same garbage for
   * as long as it rests, and re-anchoring on it would corrupt the position.
   *
   * Units are potentiometer counts; positive means increasing raw value.
   */
  class PositionUnwrapper
  {
  public:
    static constexpr int kMaxRaw = 1023;
    static constexpr double kRevolutionCounts = 1023.0 * 360.0 / 300.0; // 1227.6
    static constexpr double kRadPerCount = 2.0 * M_PI / kRevolutionCounts;

    struct Params
    {
      // Must be larger than the dead zone (~205 counts) so motion across it that was not
      // predicted (wheel turned by hand) still re-synchronises, and smaller than the distance
      // between the garbage plateau (~640) and any prediction inside the dead zone (>= 383).
      // Limit: motion that the prediction does not know about (wheel turned by hand) can be
      // up to tolerance - gap = ~45 counts per cycle (~1 rev/s at 30 Hz) and still re-sync.
      double tolerance_counts = 250.0;
      int resync_after = 0;            // 0 = never force a re-anchor
    };

    PositionUnwrapper() = default;
    explicit PositionUnwrapper(const Params &params) : params_(params) {}

    /// Start tracking at the given raw reading.
    void reset(int raw)
    {
      position_ = static_cast<double>(raw);
      rejected_in_a_row_ = 0;
      initialized_ = true;
    }

    /**
     * Feed a new reading.
     * @param raw                      raw potentiometer value (0..1023)
     * @param predicted_delta_counts   expected motion since the previous call
     * @return true if the reading was used, false if it was rejected and the
     *         position was dead-reckoned from the prediction
     */
    bool update(int raw, double predicted_delta_counts)
    {
      if (!initialized_)
      {
        reset(raw);
        return true;
      }

      const double predicted = position_ + predicted_delta_counts;

      if (raw < 0 || raw > kMaxRaw)
      {
        return reject(predicted);
      }

      const double k = std::round((predicted - raw) / kRevolutionCounts);
      const double candidate = raw + k * kRevolutionCounts;

      if (std::fabs(candidate - predicted) <= params_.tolerance_counts)
      {
        position_ = candidate;
        rejected_in_a_row_ = 0;
        return true;
      }

      if (params_.resync_after > 0 && rejected_in_a_row_ + 1 >= params_.resync_after)
      {
        // Lost for too long: trust the sensor and re-anchor to the nearest candidate.
        position_ = candidate;
        rejected_in_a_row_ = 0;
        return true;
      }
      return reject(predicted);
    }

    double position_counts() const { return position_; }
    bool initialized() const { return initialized_; }
    int rejected_in_a_row() const { return rejected_in_a_row_; }

  private:
    bool reject(double predicted)
    {
      position_ = predicted;
      ++rejected_in_a_row_;
      return false;
    }

    Params params_;
    double position_ = 0.0;
    int rejected_in_a_row_ = 0;
    bool initialized_ = false;
  };
}

#endif // DYNAMIXEL_HW__POSITION_UNWRAPPER_HPP_
