#ifndef DYNAMIXEL_HW__DYNAMIXEL_WHEEL_HPP_
#define DYNAMIXEL_HW__DYNAMIXEL_WHEEL_HPP_

#include <string>
#include <dynamixel_hw/dynamixel_connection.hpp>
#include <dynamixel_hw/position_unwrapper.hpp>

namespace dynamixel_hw
{
  class DynamixelWheel
  {
  public:
    std::string name;

    double cmd = 0;
    double pos = 0;
    double vel = 0;

    void init(const std::string name, const uint8_t dynamixel_id, bool is_inverted);
    bool setup(const DynamixelConnection &connection);
    void shutdown();

    bool rotate(double velocity);
    /// Velocity the servo was last told to run at (after quantisation and deadband) [rad/s].
    double get_velocity();

    /// Read the present position once and use it as the zero of `pos`.
    bool init_position();
    /// Read the present position, update `pos` and `vel`. Returns false if the read failed
    /// (position is then dead-reckoned from the commanded velocity).
    bool update_from_hardware(double dt);
    int consecutive_failures() const { return consecutive_failures_; }

  private:
    dynamixel_hw::DynamixelConnection connection_;

    uint8_t dynamixel_id_;
    int8_t inverted_;

    double executed_vel_ = 0.0; // [rad/s] what the servo was last told to do
    PositionUnwrapper unwrapper_;
    double anchor_counts_ = 0.0;
    double prev_pos_ = 0.0;
    int consecutive_failures_ = 0;
    bool deadband_warned_ = false;

    uint16_t convert_to_dynamixel_vel(double rad_vel);
    double convert_to_ros_control_vel(uint16_t dynamixel_vel);
  };
}

#endif