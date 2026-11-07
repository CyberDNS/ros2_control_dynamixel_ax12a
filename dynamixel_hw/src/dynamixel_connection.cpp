#include "dynamixel_hw/dynamixel_connection.hpp"
#include "rclcpp/rclcpp.hpp"

// Protocol version
#define PROTOCOL_VERSION 1.0 // Default Protocol version of DYNAMIXEL X series.

#define LOGGER_IDENTIFIER "dynamixel_hw > dynamixel_connection"

namespace dynamixel_hw
{
  bool DynamixelConnection::connect(const std::string &device, int32_t baud_rate)
  {
    portHandler = dynamixel::PortHandler::getPortHandler(device.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to open the port!");
      return false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(baud_rate);
    if (dxl_comm_result == false)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to set the baudrate!");
      return false;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger(LOGGER_IDENTIFIER), "Succeeded to set the baudrate.");
    }

    return true;
  }

  bool DynamixelConnection::write2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t value)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dynamixel_id, address, value, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to write 2 bytes.");
      return false;
    }
    return true;
  };

  bool DynamixelConnection::write1ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint8_t value)
  {

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dynamixel_id, address, value, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to write 1 bytes.");
      return false;
    }
    return true;
  };

  bool DynamixelConnection::read1ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint8_t *data)
  {
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, dynamixel_id, address, data, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read 1 byte.");
      return false;
    }
    return true;
  };

  bool DynamixelConnection::read2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t *data)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dynamixel_id, address, data, &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger(LOGGER_IDENTIFIER), "Failed to read 2 bytes.");
      return false;
    }
    return true;
  };

  void DynamixelConnection::disconnect()
  {
    if (portHandler != nullptr)
    {
      portHandler->closePort();
    }

    delete packetHandler;
    delete portHandler;
    packetHandler = nullptr;
    portHandler = nullptr;
  }

}