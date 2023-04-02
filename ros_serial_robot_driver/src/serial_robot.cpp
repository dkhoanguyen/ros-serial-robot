#include "ros_serial_robot_driver/serial_robot.hpp"

namespace RosSerialRobot
{
  SerialRobot::SerialRobot(
      const unsigned int &timeout,
      const unsigned int &num_tries)
      : timeout_(timeout), num_tries_(num_tries)
  {
  }

  SerialRobot::SerialRobot() : SerialRobot(30, 5)
  {
  }

  SerialRobot::~SerialRobot()
  {
  }

  void SerialRobot::setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port)
  {
    serial_port_ = serial_port;
  }

  int SerialRobot::readResponse(std::vector<uint8_t> &response)
  {
    std::unique_lock<std::mutex> lck(comms_mtx_);
    std::vector<uint8_t> returned_data;
    if (serial_port_->available() == 0)
    {
      return (int)SerialCommandError::NO_RESPONSE;
    }

    // To support testing this should be a separate function
    unsigned int num_byte_read = 0;
    for (int i = 0; i < num_tries_; i++)
    {
      num_byte_read = serial_port_->read(returned_data);
      // If we actually receive data
      if (num_byte_read > 0)
      {
        break;
      }
    }
    // Unable to get a response
    if (num_byte_read == 0)
    {
      return (int)SerialCommandError::READ_ERROR;
    }
    // Verify header first
    for (int i = 0; i < sample_packet_.headers.size(); i++)
    {
      if (returned_data.at(i) != sample_packet_.headers.at(i))
      {
        return (int)SerialCommandError::WRONG_HEADER;
      }
    }
    // Then verify checksum
    if (calcCheckSum(returned_data) != returned_data.at(returned_data.size() - 1))
    {
      return (int)SerialCommandError::WRONG_CHECKSUM;
    }

    response = returned_data;

    return (int)SerialCommandError::NO_ERROR;
  }

  int SerialRobot::writeCommand(const std::vector<uint8_t> &command)
  {
    std::unique_lock<std::mutex> lck(comms_mtx_);
    try
    {
      serial_port_->write(command);
      serial_port_->wait();
      return (int)SerialCommandError::NO_ERROR;
    }
    catch (const std::exception &se)
    {
      // Handle errors
      return (int)SerialCommandError::WRITE_ERROR;
    }
  }

  void SerialRobot::packFromFloats(const std::vector<float> &value_to_pack, std::vector<uint8_t> &packed_floats)
  {
    for (const float value : value_to_pack)
    {
      uint8_t bytes_temp[4];
      floatToByte(value, bytes_temp);
      for (int j = 0; j < 4; ++j)
      {
        packed_floats.push_back(bytes_temp[j]);
      }
    }
  }

  void SerialRobot::packFromDoubles(const std::vector<double> &value_to_pack, std::vector<uint8_t> &packed_doubles)
  {
    for (const double value : value_to_pack)
    {
      uint8_t bytes_temp[8];
      doubleToByte(value, bytes_temp);
      for (int j = 0; j < 8; j++)
      {
        packed_doubles.push_back(bytes_temp[j]);
      }
    }
  }

  void SerialRobot::unpackFloats(const std::vector<uint8_t>::iterator &it, float &output)
  {
    uint8_t b[] = {*it, *(it + 1), *(it + 2), *(it + 3)};
    std::memcpy(&output, &b, sizeof(output)); // convert to float from bytes[4]
                                              //     printf("%f\n", temp);
  }

  void SerialRobot::floatToByte(float float_variable, uint8_t temp_bytes[])
  {
    union
    {
      float a;
      uint8_t bytes[4];
    } link;
    link.a = float_variable;
    std::memcpy(temp_bytes, link.bytes, 4);
  }

  void SerialRobot::doubleToByte(double double_variable, uint8_t temp_bytes[])
  {
    union
    {
      double a;
      uint8_t bytes[8];
    } link;

    link.a = double_variable;
    std::memcpy(temp_bytes, link.bytes, 8);
  }
}