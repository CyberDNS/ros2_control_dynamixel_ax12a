// Copyright 2020 Yutaka Kondo <yutaka.kondo@youtalk.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DYNAMIXEL_HW__DYNAMIXEL_HW_HPP_
#define DYNAMIXEL_HW__DYNAMIXEL_HW_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dynamixel_hw/dynamixel_connection.hpp"
#include "dynamixel_hw/dynamixel_wheel.hpp"
#include "dynamixel_hw/visiblity_control.h"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace dynamixel_hw
{
  class DynamixelHw : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string left_wheel_name = "";
      std::string right_wheel_name = "";
      std::string usb_port = "";
      int baud_rate = 0;
    };

  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHw)

    DYNAMIXEL_HW_PUBLIC
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

    DYNAMIXEL_HW_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DYNAMIXEL_HW_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DYNAMIXEL_HW_PUBLIC
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

    DYNAMIXEL_HW_PUBLIC
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    DYNAMIXEL_HW_PUBLIC
    return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    DYNAMIXEL_HW_PUBLIC
    return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    dynamixel_hw::DynamixelWheel left_wheel_;
    dynamixel_hw::DynamixelWheel right_wheel_;

    dynamixel_hw::DynamixelConnection dynamixel_connection_;

    Config config_;
  };

} // namespace dynamixel_hw

#endif // DYNAMIXEL_HW__DYNAMIXEL_HW_HPP_
