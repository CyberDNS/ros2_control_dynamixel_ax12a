#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "dynamixel_hw/position_unwrapper.hpp"
#include "dynamixel_hw/velocity_controller.hpp"
#include "dynamixel_hw/velocity_estimator.hpp"

using dynamixel_hw::PositionUnwrapper;
using dynamixel_hw::VelocityController;
using dynamixel_hw::VelocityEstimator;

namespace
{
  constexpr double kDt = 1.0 / 30.0;
  constexpr double kCountRad = PositionUnwrapper::kRadPerCount;
  constexpr double kMinSpeed = 80.0 / (1023.0 / 6.283185307179586); // driver deadband [rad/s]

  /// Simulated AX-12A in wheel mode: deadband, gain != 1, offset, first-order lag.
  /// The numbers are the ones measured on the robot's left servo (stand, no load).
  struct Servo
  {
    double gain = 1.2;
    double offset = 0.3;   // rad/s lost to friction
    double tau = 0.1;      // s
    double speed = 0.0;    // true wheel speed
    double position = 0.0; // true wheel position

    void step(double command, double dt)
    {
      double steady = 0.0;
      if (std::fabs(command) >= kMinSpeed)
      {
        steady = std::copysign(std::max(0.0, gain * std::fabs(command) - offset), command);
      }
      speed += (steady - speed) * (dt / (tau + dt));
      position += speed * dt;
    }
    // what the potentiometer reading resolves to: quantised to whole counts
    double measured_position() const { return std::round(position / kCountRad) * kCountRad; }
  };

  struct Result
  {
    double mean = 0.0;
    double stddev = 0.0;
  };

  /// Run the loop and return the mean/stddev of the true speed over [t_from, t_to].
  Result run(Servo servo, double target, double seconds, double t_from, bool closed_loop,
             VelocityController::Params params = VelocityController::Params())
  {
    VelocityController ctl(params);
    VelocityEstimator est;
    est.reset(servo.measured_position());
    std::vector<double> samples;
    double command = target;
    for (int i = 0; i < static_cast<int>(seconds / kDt); ++i)
    {
      servo.step(command, kDt);
      est.push(servo.measured_position(), kDt, true);
      command = closed_loop ? ctl.update(target, est.velocity(), est.valid(), kDt) : target;
      if (i * kDt >= t_from)
      {
        samples.push_back(servo.speed);
      }
    }
    Result r;
    for (double s : samples) r.mean += s;
    r.mean /= samples.size();
    for (double s : samples) r.stddev += (s - r.mean) * (s - r.mean);
    r.stddev = std::sqrt(r.stddev / samples.size());
    return r;
  }
}

TEST(VelocityEstimator, ConstantSpeedIsRecovered)
{
  VelocityEstimator est;
  est.reset(0.0);
  double pos = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    pos += 2.0 * kDt;
    est.push(pos, kDt, true);
  }
  EXPECT_TRUE(est.valid());
  EXPECT_NEAR(est.velocity(), 2.0, 1e-9);
}

TEST(VelocityEstimator, FrozenAfterUncleanSample)
{
  VelocityEstimator est;
  est.reset(0.0);
  double pos = 0.0;
  for (int i = 0; i < 10; ++i)
  {
    pos += 2.0 * kDt;
    est.push(pos, kDt, true);
  }
  ASSERT_TRUE(est.valid());
  // a re-synchronisation jump of 0.15 rad must not leak into the estimate
  pos += 2.0 * kDt + 0.15;
  est.push(pos, kDt, false);
  EXPECT_FALSE(est.valid());
  EXPECT_NEAR(est.velocity(), 2.0, 1e-9);
  for (int i = 0; i < 3; ++i)
  {
    pos += 2.0 * kDt;
    est.push(pos, kDt, true);
    EXPECT_FALSE(est.valid());
    EXPECT_NEAR(est.velocity(), 2.0, 1e-9);
  }
  pos += 2.0 * kDt;
  est.push(pos, kDt, true);
  EXPECT_TRUE(est.valid()); // window no longer contains the jump
  EXPECT_NEAR(est.velocity(), 2.0, 1e-9);
}

TEST(VelocityEstimator, QuantisationNoiseIsSmall)
{
  Servo servo;
  servo.speed = 2.0;
  VelocityEstimator est;
  est.reset(servo.measured_position());
  double worst = 0.0;
  for (int i = 0; i < 90; ++i)
  {
    servo.position += 2.0 * kDt;
    est.push(servo.measured_position(), kDt, true);
    if (i > 10 && est.valid())
    {
      worst = std::max(worst, std::fabs(est.velocity() - 2.0));
    }
  }
  EXPECT_LT(worst, 0.06); // one count over 4 cycles = 0.04 rad/s
}

TEST(VelocityController, OpenLoopMissesTargetAtLowAndHighSpeed)
{
  // documents the problem the loop solves
  EXPECT_LT(run(Servo(), 0.83, 6.0, 2.0, false).mean, 0.83 * 0.9);
  EXPECT_GT(run(Servo(), 5.0, 6.0, 2.0, false).mean, 5.0 * 1.1);
}

TEST(VelocityController, ClosedLoopHitsTargetAtAllSpeeds)
{
  for (double target : {0.83, 1.67, 3.33, 5.0, -1.67, -5.0})
  {
    const Result r = run(Servo(), target, 6.0, 3.0, true);
    EXPECT_NEAR(r.mean, target, std::fabs(target) * 0.04) << "target " << target;
    EXPECT_LT(r.stddev, std::max(0.12, std::fabs(target) * 0.08)) << "target " << target;
  }
}

TEST(VelocityController, WorksForTheOtherServo)
{
  Servo right;
  right.gain = 1.1;
  right.offset = 0.2;
  for (double target : {0.83, 1.67, 3.33, 5.0})
  {
    const Result r = run(right, target, 6.0, 3.0, true);
    EXPECT_NEAR(r.mean, target, target * 0.04) << "target " << target;
  }
}

TEST(VelocityController, TwoDifferentServosEndUpAtTheSameSpeed)
{
  Servo left;
  Servo right;
  right.gain = 1.05;
  right.offset = 0.1;
  const double l = run(left, 2.5, 6.0, 3.0, true).mean;
  const double r = run(right, 2.5, 6.0, 3.0, true).mean;
  EXPECT_NEAR(l, r, 0.1);
}

TEST(VelocityController, StopIsImmediateAndClearsTheIntegral)
{
  VelocityController ctl;
  for (int i = 0; i < 60; ++i)
  {
    ctl.update(2.0, 1.0, true, kDt); // sustained error builds the integral
  }
  EXPECT_GT(ctl.integral(), 0.1);
  EXPECT_DOUBLE_EQ(ctl.update(0.0, 1.0, true, kDt), 0.0);
  EXPECT_DOUBLE_EQ(ctl.integral(), 0.0);
}

TEST(VelocityController, ReversalClearsTheIntegral)
{
  VelocityController ctl;
  for (int i = 0; i < 60; ++i)
  {
    ctl.update(2.0, 1.0, true, kDt);
  }
  const double out = ctl.update(-2.0, 0.0, true, kDt);
  EXPECT_LT(out, -1.9); // no leftover positive correction
}

TEST(VelocityController, StaleMeasurementDoesNotWindUp)
{
  VelocityController ctl;
  for (int i = 0; i < 300; ++i)
  {
    ctl.update(2.0, 0.0, false, kDt); // no valid measurement
  }
  EXPECT_DOUBLE_EQ(ctl.integral(), 0.0);
  EXPECT_DOUBLE_EQ(ctl.update(2.0, 0.0, false, kDt), 2.0); // pure feed-forward
}

TEST(VelocityController, IntegralIsLimited)
{
  VelocityController::Params p;
  VelocityController ctl(p);
  for (int i = 0; i < 3000; ++i)
  {
    ctl.update(2.0, 0.0, true, kDt); // stalled wheel
  }
  EXPECT_LE(p.ki * ctl.integral(), p.max_correction + 1e-9);
  EXPECT_LE(ctl.update(2.0, 0.0, true, kDt), p.max_output);
}

namespace
{
  /// Two different servos follow the same ramp; return the largest position difference [rad].
  double max_position_gap(bool position_tracking)
  {
    Servo left;
    Servo right;
    left.gain = 1.05;   // slower, more friction
    left.offset = 0.5;
    right.gain = 1.2;
    right.offset = 0.2;
    VelocityController::Params p;
    if (!position_tracking)
    {
      p.kpos = 0.0;
    }
    VelocityController cl(p), cr(p);
    VelocityEstimator el, er;
    el.reset(0.0);
    er.reset(0.0);
    double cmd_l = 0.0, cmd_r = 0.0, worst = 0.0;
    for (int i = 0; i < static_cast<int>(6.0 / kDt); ++i)
    {
      const double t = i * kDt;
      const double target = std::min(3.3, 3.3 * t / 1.0); // 1 s acceleration ramp
      left.step(cmd_l, kDt);
      right.step(cmd_r, kDt);
      el.push(left.measured_position(), kDt, true);
      er.push(right.measured_position(), kDt, true);
      cmd_l = cl.update(target, el.velocity(), el.valid(), kDt, left.measured_position(), true);
      cmd_r = cr.update(target, er.velocity(), er.valid(), kDt, right.measured_position(), true);
      worst = std::max(worst, std::fabs(left.position - right.position));
    }
    return worst;
  }
}

TEST(VelocityController, PositionTrackingKeepsWheelsInStep)
{
  const double without = max_position_gap(false);
  const double with = max_position_gap(true);
  EXPECT_LT(with, without * 0.6) << "without " << without << " with " << with;
}

TEST(VelocityController, PositionReferenceIsBounded)
{
  VelocityController::Params p;
  VelocityController ctl(p);
  double out = 0.0;
  for (int i = 0; i < 3000; ++i)
  {
    out = ctl.update(2.0, 0.0, true, kDt, 0.0, true); // blocked wheel
  }
  EXPECT_LE(out, p.max_output);
}

TEST(VelocityController, PositionReferenceRestartsAfterStop)
{
  VelocityController ctl;
  for (int i = 0; i < 30; ++i) ctl.update(2.0, 2.0, true, kDt, i * 2.0 * kDt, true);
  ctl.update(0.0, 0.0, true, kDt, 2.0, true);
  // a new motion starts from the current position: no kick from the old reference
  const double out = ctl.update(2.0, 0.0, false, kDt, 2.0, true);
  EXPECT_NEAR(out, 2.0, 0.3); // at most one cycle of lead: kpos * target * dt = 0.27
}
