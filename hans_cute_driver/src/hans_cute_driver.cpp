#include "hans_cute_driver/hans_cute_driver.hpp"

namespace HansCuteRobot
{
  HansCuteDriver::HansCuteDriver()
  {
  }

  HansCuteDriver::~HansCuteDriver()
  {
  }

  bool HansCuteDriver::configure()
  {
    std::cout << "Hans Cute Driver" << std::endl;
  }

  uint8_t HansCuteDriver::calcCheckSum(std::vector<uint8_t> &data) const
  {
    unsigned int checksum = 0;
    unsigned int payload_sum = 0;
    unsigned int bytes_to_read = data.at(sample_packet_.length); // Add ID and length as well

    for (int i = 2; i <= 2 + bytes_to_read; i++)
    {
      payload_sum += (int)data.at(i);
    }
    checksum = 255 - (payload_sum % 256);
    return (uint8_t)checksum;
  }

  bool HansCuteDriver::getJointsPosition(std::vector<double> &positions)
  {
  }

  bool HansCuteDriver::setJointsPosition(const std::vector<double> positions)
  {
  }

  // PROTECTED //

  bool HansCuteDriver::read(const uint8_t &id, const uint8_t &address, const uint8_t &size,
                            std::vector<uint8_t> &returned_data, unsigned long &timestamp)
  {
    // Number of bytes following standard header (0xFF, 0xFF, id, length)
    uint8_t length = 4;
    std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::READ_DATA, address, size};

    // directly from AX-12 manual:
    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::READ_DATA + address + size) % 256);
    packet.push_back(checksum);

    // Thread safe execution of
    if (!writeCommand(packet))
    {
      return false;
    }
    // Read response ans return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }

  bool HansCuteDriver::write(const uint8_t &id, const uint8_t &address, const std::vector<uint8_t> &data,
                             std::vector<uint8_t> &returned_data, unsigned long &timestamp)
  {
    // Number of bytes following standard header (0xFF, 0xFF, id, length)
    uint8_t length = 3 + (uint8_t)data.size();
    std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::WRITE_DATA, address};
    packet.insert(std::end(packet), std::begin(data), std::end(data));

    // directly from AX-12 manual:
    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    unsigned int sum = 0;
    for (uint8_t data_point : data)
    {
      sum += data_point;
    }
    sum += (id + length + (uint8_t)InstructionSet::WRITE_DATA + address);
    uint8_t checksum = 255 - (sum % 256);
    packet.push_back(checksum);

    // Thread safe execution of
    if (!writeCommand(packet))
    {
      std::cout << "Failed to write packet" << std::endl;
      return false;
    }
    // Read response ans return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }

  bool HansCuteDriver::syncWrite(const uint8_t &address, const std::vector<std::vector<uint8_t>> &data)
  {
    // First flatten the input data
    std::vector<uint8_t> flatten_data;
    unsigned int sum = 0;
    for (std::vector<uint8_t> servo : data)
    {
      for (uint8_t data_point : servo)
      {
        flatten_data.push_back(data_point);
        sum += data_point;
      }
    }
    // Number of bytes following standard header (0xFF, 0xFF, id, length) plus data
    uint8_t length = 4 + flatten_data.size();
    uint8_t servo_data_length = data.at(0).size() - 1;
    std::vector<uint8_t> packet = {0xFF, 0xFF, (uint8_t)BroadcastConstant::BROADCAST, length, (uint8_t)InstructionSet::SYNC_WRITE, address, servo_data_length};
    packet.insert(std::end(packet), std::begin(flatten_data), std::end(flatten_data));

    sum += (uint8_t)BroadcastConstant::BROADCAST + length + (uint8_t)InstructionSet::SYNC_WRITE + address + servo_data_length;
    uint8_t checksum = 255 - (sum % 256);
    packet.push_back(checksum);

    // Thread safe execution of
    if (!writeCommand(packet))
    {
      return false;
    }
    // Read response ans return data
    std::vector<uint8_t> returned_data;
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    return true;
  }

  bool HansCuteDriver::ping(const uint8_t &id, std::vector<uint8_t> &returned_data)
  {
    uint8_t length = 2;
    std::vector<uint8_t> packet = {0xFF, 0xFF, id, length, (uint8_t)InstructionSet::PING};

    // directly from AX-12 manual:
    // Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
    // If the calculated value is > 255, the lower byte is the check sum.
    // Don't ask me why
    uint8_t checksum = 255 - ((id + length + (uint8_t)InstructionSet::PING) % 256);
    packet.push_back(checksum);

    // Thread safe execution of
    if (!writeCommand(packet))
    {
      return false;
    }
    // Read response ans return data
    int status = readResponse(returned_data);
    if (status != 0)
    {
      return false;
    }
    // We need to handle error code returned from the robot
    return true;
  }

  //====================================================================//
  // These function modify EEPROM data which persists after power cycle //
  //====================================================================//

  // These can be left until the end as they are relatively unimportant
  bool HansCuteDriver::setID(const uint8_t &old_id, const uint8_t &new_id)
  {
    return true;
  }
  bool HansCuteDriver::setBaudrate(const uint8_t &servo_id, const long &baudrate)
  {
    return true;
  }

  bool HansCuteDriver::setReturnDelayTime(const uint8_t &servo_id, const unsigned int &delay_time)
  {
    return true;
  }
  bool HansCuteDriver::setAngleLimits(const uint8_t &servo_id, const unsigned int &min_limit,
                                      const unsigned int &max_limit)
  {
    std::vector<uint8_t> raw_limit = HansCuteRobot::AngleLimits::getRawData(min_limit, max_limit);
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L, raw_limit, returned_data, timestamp);
    return true;
  }

  bool HansCuteDriver::setVoltageLimits(const uint8_t &servo_id, const VoltageLimits &voltage_limits)
  {
    return true;
  }

  bool HansCuteDriver::setMaxTorque(const uint8_t &servo_id, const unsigned int &max_torque)
  {
    std::vector<uint8_t> raw_limit = HansCuteRobot::TorqueLimit::getRawData(max_torque);
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::MAX_TORQUE_L, raw_limit, returned_data, timestamp);
    return true;
  }

  //===============================================================//
  // These functions can send multiple commands to a single servo  //
  //===============================================================//
  bool HansCuteDriver::setTorqueEnable(const uint8_t &servo_id, const bool &enabled)
  {
    std::vector<uint8_t> data({(uint8_t)enabled});
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::TORQUE_ENABLE, data, returned_data, timestamp);
    return true;
  }
  bool HansCuteDriver::setComplianceMargin()
  {
    return true;
  }
  bool HansCuteDriver::setComplianceSlope()
  {
    return true;
  }
  bool HansCuteDriver::setDGain()
  {
    return true;
  }
  bool HansCuteDriver::setIGain()
  {
    return true;
  }
  bool HansCuteDriver::setPGain()
  {
    return true;
  }

  bool HansCuteDriver::setAcceleration(const uint8_t &servo_id, const unsigned int &acceleration)
  {
    std::vector<uint8_t> raw_data = HansCuteRobot::ServoAcceleration::getRawData(acceleration);
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::GOAL_ACCELERATION, raw_data, returned_data, timestamp);
    return true;
  }
  bool HansCuteDriver::setPosition(const uint8_t &servo_id, const unsigned int &position)
  {
    std::vector<uint8_t> raw_postion = HansCuteRobot::ServoPosition::getRawData(position);
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::GOAL_POSITION_L, raw_postion, returned_data, timestamp);
    return true;
  }
  bool HansCuteDriver::setSpeed(const uint8_t &servo_id, const unsigned int &speed)
  {
    std::vector<uint8_t> raw_speed = HansCuteRobot::ServoSpeed::getRawData(speed);
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::GOAL_SPEED_L, raw_speed, returned_data, timestamp);
    return true;
  }

  bool HansCuteDriver::setTorqueLimit(const uint8_t &servo_id, const unsigned int &torque_limit)
  {
    std::vector<uint8_t> raw_torque = HansCuteRobot::TorqueLimit::getRawData(torque_limit);
    std::vector<uint8_t> returned_data;
    unsigned long timestamp;
    write(servo_id, (uint8_t)ControlTableConstant::TORQUE_LIMIT_L, raw_torque, returned_data, timestamp);
    return true;
  }
  bool HansCuteDriver::setGoalTorque()
  {
    return true;
  }

  bool HansCuteDriver::setPositionAndSpeed()
  {
    return true;
  }

  //===============================================================//
  // These functions can send multiple commands to multiple servos //
  //===============================================================//

  bool HansCuteDriver::setMultiTorqueEnabled()
  {
    return true;
  }

  // Range is between 0 -> 255
  bool HansCuteDriver::setMultiComplianceMargin()
  {
    return true;
  }
  bool HansCuteDriver::setMultiComplianceSlope()
  {
    return true;
  }

  // Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
  bool HansCuteDriver::setMultiPosition(const std::vector<unsigned int> &servo_ids,
                                        const std::vector<unsigned int> &positions)
  {
  }
  bool HansCuteDriver::setMultiSpeed(const std::vector<unsigned int> &servo_ids,
                                     const std::vector<unsigned int> &speeds)
  {
  }

  bool HansCuteDriver::setMultiTorqueLimit(const std::vector<unsigned int> &servo_ids,
                                           const std::vector<double> &torque_limits)
  {
    return true;
  }
  bool HansCuteDriver::setMultiPositionAndSpeed(const std::vector<unsigned int> &servo_ids,
                                                const std::vector<unsigned int> &positions,
                                                const std::vector<unsigned int> &speeds)
  {
    std::vector<std::vector<uint8_t>> data_lists;
    for (int indx = 0; indx < servo_ids.size(); indx++)
    {
      std::vector<uint8_t> data;
      data.push_back((uint8_t)servo_ids.at(indx));
      std::vector<uint8_t> position = HansCuteRobot::ServoPosition::getRawData(positions.at(indx));
      data.insert(data.end(), position.begin(), position.end());
      std::vector<uint8_t> speed = HansCuteRobot::ServoSpeed::getRawData(speeds.at(indx));
      data.insert(data.end(), speed.begin(), speed.end());
      data_lists.push_back(data);
    }
    syncWrite((uint8_t)ControlTableConstant::GOAL_POSITION_L, data_lists);
    return true;
  }

  //===============================//
  // Servo status access functions //
  //===============================//

  bool HansCuteDriver::getModelNumber(const int &servo_id, unsigned int &model_number)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::MODEL_NUMBER_L, 2, response, timestamp))
    {
      return false;
    }
    model_number = HansCuteRobot::ModelNumber::getData(response);
    return true;
  }
  bool HansCuteDriver::getFirmwareVersion()
  {
    return true;
  }
  bool HansCuteDriver::getReturnDelayTime()
  {
    return true;
  }

  bool HansCuteDriver::getAngleLimits(const int &servo_id, HansCuteRobot::AngleLimits &angle_limit)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::CW_ANGLE_LIMIT_L, 4, response, timestamp))
    {
      return false;
    }
    angle_limit = HansCuteRobot::AngleLimits::getData(response);
    return true;
  }

  bool HansCuteDriver::getTorqueEnabled(const int &servo_id, bool &enabled)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::TORQUE_ENABLE, 1, response, timestamp))
    {
      return false;
    }
    return true;
  }

  bool HansCuteDriver::getMaxTorque(const int &servo_id, unsigned int &torque)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::MAX_TORQUE_L, 2, response, timestamp))
    {
      return false;
    }
    torque = HansCuteRobot::TorqueLimit::getData(response);
    return true;
  }

  bool HansCuteDriver::getVoltageLimits()
  {
    return true;
  }
  bool HansCuteDriver::getPosition(const int &servo_id, unsigned int &position)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::PRESENT_POSITION_L, 2, response, timestamp))
    {
      return false;
    }
    position = (unsigned int)HansCuteRobot::ServoPosition::getData(response);
    return true;
  }
  bool HansCuteDriver::getSpeed(const int &servo_id, unsigned int &speed)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::GOAL_SPEED_L, 2, response, timestamp))
    {
      return false;
    }
    speed = (unsigned int)HansCuteRobot::ServoSpeed::getData(response);
    return true;
  }

  bool HansCuteDriver::getAcceleration(const int &servo_id, unsigned int &acceleration)
  {
    return true;
  }

  bool HansCuteDriver::getVoltage()
  {
    return true;
  }
  bool HansCuteDriver::getCurrent()
  {
    return true;
  }

  bool HansCuteDriver::getLock(const int &servo_id, bool &lock)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::LOCK, 1, response, timestamp))
    {
      return false;
    }
    lock = (bool)response.at(5);
    return true;
  }

  bool HansCuteDriver::getFeedback(const int &servo_id, ServoFeedback &feedback)
  {
    std::vector<uint8_t> response;
    unsigned long timestamp = 0;
    if (!read((uint8_t)servo_id, (uint8_t)ControlTableConstant::GOAL_POSITION_L, 17, response, timestamp))
    {
      return false;
    }

    feedback = ServoFeedback::getData((uint8_t)servo_id, response, timestamp);
    return true;
  }
} // namespace HansCuteRobot

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HansCuteRobot::HansCuteDriver, RosSerialRobot::SerialRobot)