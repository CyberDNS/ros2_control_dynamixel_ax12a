#include <gtest/gtest.h>

#include "dynamixel_hw/voltage_monitor.hpp"

using dynamixel_hw::VoltageMonitor;
using State = VoltageMonitor::State;

TEST(VoltageMonitor, NormalVoltageIsOk)
{
  VoltageMonitor m;
  EXPECT_EQ(m.update(12.6), State::kOk);
  EXPECT_EQ(m.update(11.1), State::kOk);
}

TEST(VoltageMonitor, LowVoltageWarns)
{
  VoltageMonitor m;
  EXPECT_EQ(m.update(10.2), State::kWarn);
  EXPECT_EQ(m.update(10.2), State::kWarn);
}

TEST(VoltageMonitor, StopOnlyAfterConsecutiveReadings)
{
  VoltageMonitor m;
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(9.0), State::kStop);
}

TEST(VoltageMonitor, ASingleDipIsForgiven)
{
  VoltageMonitor m;
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(11.5), State::kOk); // recovered: counter reset
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(9.0), State::kStop);
}

TEST(VoltageMonitor, ImplausibleReadingsAreIgnored)
{
  VoltageMonitor m;
  EXPECT_EQ(m.update(0.0), State::kOk);
  EXPECT_EQ(m.update(-1.0), State::kOk);
  EXPECT_EQ(m.update(255.0), State::kOk);
  // and they do not count towards a stop
  EXPECT_EQ(m.update(9.0), State::kWarn);
  EXPECT_EQ(m.update(0.0), State::kOk);
  EXPECT_EQ(m.update(9.0), State::kWarn);
}

TEST(VoltageMonitor, ThresholdsCanBeDisabled)
{
  VoltageMonitor::Params p;
  p.warn_volts = 0.0;
  p.stop_volts = 0.0;
  VoltageMonitor m(p);
  EXPECT_EQ(m.update(5.0), State::kOk);
}
