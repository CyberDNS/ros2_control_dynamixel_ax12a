#include "dynamixel_hw/dynamixel_hw.hpp"

#include <algorithm>
#include <chrono>
#include <optional>
#include <string>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "dynamixel_hw/dynamixel_connection.hpp"
#include "dynamixel_hw/dynamixel_wheel.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hw
{

  CallbackReturn DynamixelHw::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "configure");
    if (hardware_interface::HardwareComponentInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

  const auto & hp = params.hardware_info.hardware_parameters;

  auto get_required_str = [&](const char* key) -> std::optional<std::string> {
    auto it = hp.find(key);
    if (it == hp.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("dynamixel_hw"), "Missing required param '%s'", key);
      return std::nullopt;
    }
    return it->second;
  };

  auto get_required_int = [&](const char* key) -> std::optional<int> {
    auto s = get_required_str(key);
    if (!s) return std::nullopt;
    try { return std::stoi(*s); }
    catch (...) {
      RCLCPP_ERROR(rclcpp::get_logger("dynamixel_hw"),
                   "Param '%s' must be an integer, got '%s'", key, s->c_str());
      return std::nullopt;
    }
  };

  // --- read your params (names as you used them) ---
  auto usb_port      = get_required_str("usb_port");
  auto baud_rate     = get_required_int("baud_rate");
  auto left_name     = get_required_str("left_wheel_name");
  auto right_name    = get_required_str("right_wheel_name");

  if (!usb_port || !baud_rate || !left_name || !right_name) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // store
  config_.usb_port        = *usb_port;
  config_.baud_rate       = *baud_rate;
  config_.left_wheel_name = *left_name;
  config_.right_wheel_name= *right_name;

  // optional: position_feedback (default true)
  if (auto it = hp.find("position_feedback"); it != hp.end()) {
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c){ return std::tolower(c); });
    config_.position_feedback = !(v == "false" || v == "0" || v == "no");
  }
  auto get_bool = [&](const char *key, bool fallback) {
    auto it = hp.find(key);
    if (it == hp.end()) return fallback;
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c){ return std::tolower(c); });
    return !(v == "false" || v == "0" || v == "no");
  };
  auto get_double = [&](const char *key, double fallback) {
    auto it = hp.find(key);
    if (it == hp.end()) return fallback;
    try { return std::stod(it->second); } catch (...) { return fallback; }
  };
  config_.velocity_loop = config_.position_feedback && get_bool("velocity_loop", true);
  config_.velocity_kp = get_double("velocity_kp", config_.velocity_kp);
  config_.velocity_ki = get_double("velocity_ki", config_.velocity_ki);
  config_.min_voltage_warn = get_double("min_voltage_warn", config_.min_voltage_warn);
  config_.min_voltage_stop = get_double("min_voltage_stop", config_.min_voltage_stop);
  {
    VoltageMonitor::Params mp;
    mp.warn_volts = config_.min_voltage_warn;
    mp.stop_volts = config_.min_voltage_stop;
    voltage_monitor_ = VoltageMonitor(mp);
  }
  RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "position_feedback=%s velocity_loop=%s (kp=%.2f ki=%.2f)",
              config_.position_feedback ? "true" : "false (open loop)",
              config_.velocity_loop ? "true" : "false", config_.velocity_kp, config_.velocity_ki);
  {
    VelocityController::Params vp;
    vp.kp = config_.velocity_kp;
    vp.ki = config_.velocity_ki;
    left_wheel_.configure_velocity_loop(config_.velocity_loop, vp);
    right_wheel_.configure_velocity_loop(config_.velocity_loop, vp);
  }


    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DynamixelHw"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.name == config_.left_wheel_name)
      {
        std::string boolStr = joint.parameters.at("invert");
        std::transform(boolStr.begin(), boolStr.end(), boolStr.begin(), [](unsigned char c){ return std::tolower(c); });

        left_wheel_.init(joint.name, std::stoi(joint.parameters.at("id")), boolStr == "true");
      }
      if (joint.name == config_.right_wheel_name)
      {
        std::string boolStr = joint.parameters.at("invert");
        std::transform(boolStr.begin(), boolStr.end(), boolStr.begin(), [](unsigned char c){ return std::tolower(c); });

        right_wheel_.init(joint.name, std::stoi(joint.parameters.at("id")), boolStr == "true");
      }
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> DynamixelHw::export_state_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_POSITION, &left_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.vel));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_POSITION, &right_wheel_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.vel));

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DynamixelHw::export_command_interfaces()
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        left_wheel_.name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.cmd));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        right_wheel_.name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.cmd));

    return command_interfaces;
  }

  CallbackReturn DynamixelHw::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "start");

    if (!dynamixel_connection_.connect(config_.usb_port, config_.baud_rate))
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Could not open %s", config_.usb_port.c_str());
      dynamixel_connection_.disconnect();
      return CallbackReturn::ERROR;
    }

    if (!left_wheel_.setup(dynamixel_connection_) || !right_wheel_.setup(dynamixel_connection_))
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Servo setup failed (power, wiring or IDs?)");
      left_wheel_.shutdown();
      right_wheel_.shutdown();
      dynamixel_connection_.disconnect();
      return CallbackReturn::ERROR;
    }

    if (config_.position_feedback && !(left_wheel_.init_position() && right_wheel_.init_position()))
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Could not read the servo positions");
      left_wheel_.shutdown();
      right_wheel_.shutdown();
      dynamixel_connection_.disconnect();
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHw::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
  {
    RCLCPP_INFO(rclcpp::get_logger("DynamixelHw"), "stop");

    left_wheel_.shutdown();
    right_wheel_.shutdown();

    dynamixel_connection_.disconnect();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn DynamixelHw::on_error(const rclcpp_lifecycle::State & /* previous_state */)
  {
    // A servo keeps its last speed command when the controller goes away, so make sure
    // it is stopped before the component is taken out of service.
    RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "error state: stopping the wheels");
    left_wheel_.stop();
    right_wheel_.stop();
    left_wheel_.shutdown();
    right_wheel_.shutdown();
    return CallbackReturn::SUCCESS;
  }

  return_type DynamixelHw::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    const double delta_seconds = period.seconds();

    // Servo supply voltage, about once per second
    seconds_since_voltage_check_ += delta_seconds;
    if (seconds_since_voltage_check_ >= 1.0)
    {
      seconds_since_voltage_check_ = 0.0;
      double vl = 0.0, vr = 0.0;
      const bool okl = left_wheel_.read_voltage(vl);
      const bool okr = right_wheel_.read_voltage(vr);
      if (okl || okr)
      {
        const double volts = (okl && okr) ? std::min(vl, vr) : (okl ? vl : vr);
        switch (voltage_monitor_.update(volts))
        {
        case VoltageMonitor::State::kStop:
          RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"),
                       "Servo supply voltage %.1f V is below the stop limit (%.1f V): stopping the wheels",
                       volts, config_.min_voltage_stop);
          left_wheel_.stop();
          right_wheel_.stop();
          return return_type::ERROR;
        case VoltageMonitor::State::kWarn:
          if (voltage_warnings_++ % 10 == 0) // every ~10 s
          {
            RCLCPP_WARN(rclcpp::get_logger("DynamixelHw"),
                        "Low servo supply voltage: %.1f V (left %.1f, right %.1f; warn below %.1f V)",
                        volts, vl, vr, config_.min_voltage_warn);
          }
          break;
        case VoltageMonitor::State::kOk:
          voltage_warnings_ = 0;
          break;
        }
      }
    }

    if (config_.position_feedback)
    {
      // Real wheel position from the servo potentiometers
      left_wheel_.update_from_hardware(delta_seconds);
      right_wheel_.update_from_hardware(delta_seconds);

      constexpr int kMaxConsecutiveReadFailures = 10; // ~0.3 s at 30 Hz
      if (left_wheel_.consecutive_failures() >= kMaxConsecutiveReadFailures ||
          right_wheel_.consecutive_failures() >= kMaxConsecutiveReadFailures)
      {
        RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"),
                     "Lost communication with the servos (%d / %d failed reads in a row)",
                     left_wheel_.consecutive_failures(), right_wheel_.consecutive_failures());
        return return_type::ERROR;
      }
    }
    else
    {
      // Open loop: integrate the commanded (executed) velocity
      left_wheel_.vel = left_wheel_.get_velocity();
      left_wheel_.pos += left_wheel_.vel * delta_seconds;
      right_wheel_.vel = right_wheel_.get_velocity();
      right_wheel_.pos += right_wheel_.vel * delta_seconds;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("DynamixelHw"), "L: %f  R: %f", left_wheel_.pos, right_wheel_.pos);

    return return_type::OK;
  }

  return_type DynamixelHw::write(const rclcpp::Time & /* time */, const rclcpp::Duration &period)
  {
    bool is_success = false;
    const double dt = period.seconds();

    is_success = left_wheel_.drive(left_wheel_.cmd, dt);
    if (!is_success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Failed to rotate left wheel.");
    }

    is_success = right_wheel_.drive(right_wheel_.cmd, dt);
    if (!is_success)
    {
      RCLCPP_ERROR(rclcpp::get_logger("DynamixelHw"), "Failed to rotate right wheel.");
    }

    return return_type::OK;
  }

} // namespace dynamixel_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hw::DynamixelHw, hardware_interface::SystemInterface)
