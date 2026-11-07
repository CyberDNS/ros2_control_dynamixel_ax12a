#ifndef DYNAMIXEL_HW__DYNAMIXEL_CONNECTION_HPP_
#define DYNAMIXEL_HW__DYNAMIXEL_CONNECTION_HPP_

#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_hw
{
  class DynamixelConnection
  {
  public:
    bool connect(const std::string &device, int32_t baud_rate);
    void disconnect();

    bool write2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t value);
    bool write1ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint8_t value);
    bool read1ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint8_t *data);
    bool read2ByteTxRx(uint8_t dynamixel_id, uint16_t address, uint16_t *data);

  private:
    dynamixel::PortHandler *portHandler = nullptr;
    dynamixel::PacketHandler *packetHandler = nullptr;

    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
  };
}

#endif