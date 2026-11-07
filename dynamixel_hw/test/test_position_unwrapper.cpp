#include <gtest/gtest.h>

#include <cmath>

#include "dynamixel_hw/position_unwrapper.hpp"

using dynamixel_hw::PositionUnwrapper;

namespace
{
  constexpr double kRev = PositionUnwrapper::kRevolutionCounts;

  // What the potentiometer reports for a true (unwrapped) position:
  // valid raw value on the track, a constant garbage value in the dead zone
  // (the robot showed ~640 there).
  int sensor(double true_counts)
  {
    double m = std::fmod(true_counts, kRev);
    if (m < 0)
    {
      m += kRev;
    }
    if (m <= PositionUnwrapper::kMaxValidRaw)
    {
      return static_cast<int>(std::lround(m));
    }
    // dead zone as measured: plateau at 1023, garbage, plateau at 0
    if (m < 1110.0) return 1023;
    if (m < 1140.0) return 640;
    return 0;
  }

  // Drive a motion profile. The wheel really moves `step` counts per cycle while the
  // controller predicted `predicted_step`. Returns the largest absolute error seen.
  double run(PositionUnwrapper &u, double &truth, int cycles, double step, double predicted_step)
  {
    double worst = 0.0;
    for (int i = 0; i < cycles; ++i)
    {
      truth += step;
      u.update(sensor(truth), predicted_step);
      worst = std::max(worst, std::fabs(u.position_counts() - truth));
    }
    return worst;
  }
}

TEST(PositionUnwrapper, FirstReadingAnchors)
{
  PositionUnwrapper u;
  u.update(830, 0.0);
  EXPECT_TRUE(u.initialized());
  EXPECT_DOUBLE_EQ(u.position_counts(), 830.0);
}

TEST(PositionUnwrapper, ForwardThroughManyRevolutionsAndDeadZones)
{
  PositionUnwrapper u;
  double truth = 300.0;
  u.reset(sensor(truth));
  // 30 counts/cycle for 5 revolutions: crosses the dead zone several times
  const double worst = run(u, truth, 5 * 41, 30.0, 30.0);
  EXPECT_LT(worst, 1.5);
  EXPECT_GT(u.position_counts(), 300.0 + 5 * kRev - 60);
}

TEST(PositionUnwrapper, BackwardThroughDeadZones)
{
  PositionUnwrapper u;
  double truth = 900.0;
  u.reset(sensor(truth));
  const double worst = run(u, truth, 4 * 41, -30.0, -30.0);
  EXPECT_LT(worst, 1.5);
  EXPECT_LT(u.position_counts(), 900.0 - 4 * kRev + 60);
}

TEST(PositionUnwrapper, SlowSpeedCrossingDeadZone)
{
  PositionUnwrapper u;
  double truth = 1000.0;
  u.reset(sensor(truth));
  // 3 counts/cycle: the dead zone (~205 counts) lasts ~70 cycles of garbage readings
  const double worst = run(u, truth, 150, 3.0, 3.0);
  EXPECT_LT(worst, 1.5);
}

TEST(PositionUnwrapper, WheelStalledWhileCommanded)
{
  PositionUnwrapper u;
  double truth = 500.0;
  u.reset(sensor(truth));
  // controller thinks 20 counts/cycle, wheel does not move: no drift may accumulate
  const double worst = run(u, truth, 200, 0.0, 20.0);
  EXPECT_LT(worst, 21.0);
  EXPECT_NEAR(u.position_counts(), 500.0, 1.0);
}

TEST(PositionUnwrapper, WheelTurnedByHandWithoutCommand)
{
  PositionUnwrapper u;
  double truth = 100.0;
  u.reset(sensor(truth));
  // Moved 10 counts/cycle (~0.25 rev/s at 30 Hz) by hand, command is zero. While the wiper is in the dead zone the
  // position cannot be known (error up to the gap width), but the first valid reading after it
  // must pull the estimate back exactly.
  const double worst = run(u, truth, 400, 10.0, 0.0);
  EXPECT_LT(worst, 215.0);
  // end of run: look at several cycles after a valid reading
  for (int i = 0; i < 5; ++i)
  {
    truth += 0.0;
    u.update(sensor(truth), 0.0);
  }
  EXPECT_NEAR(u.position_counts(), truth, 1.5);
}

TEST(PositionUnwrapper, GarbageReadingIsRejected)
{
  PositionUnwrapper u;
  u.reset(1000);
  // in the dead zone the sensor says 640; the prediction (~1010) must win
  EXPECT_FALSE(u.update(640, 10.0));
  EXPECT_NEAR(u.position_counts(), 1010.0, 1e-9);
  EXPECT_EQ(u.rejected_in_a_row(), 1);
}

TEST(PositionUnwrapper, SaturatedEndValuesAreRejected)
{
  PositionUnwrapper u;
  u.reset(1000);
  EXPECT_FALSE(u.update(1023, 15.0));
  EXPECT_NEAR(u.position_counts(), 1015.0, 1e-9);
  EXPECT_FALSE(u.update(0, 15.0));
  EXPECT_NEAR(u.position_counts(), 1030.0, 1e-9);
  EXPECT_TRUE(u.update(5, 0.0)); // first real reading after the dead zone
  EXPECT_NEAR(u.position_counts(), 5.0 + PositionUnwrapper::kRevolutionCounts, 1e-9);
}

TEST(PositionUnwrapper, OutOfRangeReadingIsRejected)
{
  PositionUnwrapper u;
  u.reset(500);
  EXPECT_FALSE(u.update(4000, 0.0));
  EXPECT_FALSE(u.update(-3, 0.0));
  EXPECT_NEAR(u.position_counts(), 500.0, 1e-9);
}

TEST(PositionUnwrapper, ResynchronisesAfterBeingLostWhenEnabled)
{
  PositionUnwrapper::Params params;
  params.resync_after = 20;
  PositionUnwrapper u(params);
  u.reset(500);
  // wheel suddenly jumps 400 counts away while no motion is predicted
  int accepted_at = -1;
  for (int i = 0; i < 40; ++i)
  {
    if (u.update(900, 0.0) && accepted_at < 0)
    {
      accepted_at = i;
    }
  }
  EXPECT_EQ(accepted_at, 19); // 20th consecutive rejection forces a re-anchor
  EXPECT_NEAR(u.position_counts(), 900.0, 1e-9);
}

TEST(PositionUnwrapper, NeverForcesResyncByDefault)
{
  PositionUnwrapper u;
  u.reset(500);
  for (int i = 0; i < 500; ++i)
  {
    EXPECT_FALSE(u.update(900, 0.0)); // 400 counts away: always rejected
  }
  EXPECT_NEAR(u.position_counts(), 500.0, 1e-9);
}

TEST(PositionUnwrapper, DeadZonePlateauNeverAccepted)
{
  // For every prediction inside the dead zone, a garbage reading of 600..680 must be rejected.
  for (double pred = 1024.0; pred < PositionUnwrapper::kRevolutionCounts; pred += 1.0)
  {
    for (int garbage = 600; garbage <= 680; garbage += 10)
    {
      PositionUnwrapper u;
      u.reset(static_cast<int>(pred - 5));
      u.update(1023, 0.0); // sensor at the edge of the track
      PositionUnwrapper v;
      v.reset(static_cast<int>(std::floor(pred)) > 1023 ? 1023 : 1000);
      EXPECT_FALSE(v.update(garbage, pred - v.position_counts()))
          << "pred=" << pred << " garbage=" << garbage;
    }
  }
}

TEST(PositionUnwrapper, NoiseAtRestIsHarmless)
{
  PositionUnwrapper u;
  u.reset(830);
  for (int i = 0; i < 100; ++i)
  {
    u.update(830 + (i % 3) - 1, 0.0);
  }
  EXPECT_NEAR(u.position_counts(), 830.0, 1.0);
}
