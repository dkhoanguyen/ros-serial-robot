#ifndef ROS_SERIAL_ROBOT__SERIAL_ROBOT_HPP_
#define ROS_SERIAL_ROBOT__SERIAL_ROBOT_HPP_

#include <iostream>
#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <limits>
#include <cmath>
#include <thread>
#include <atomic>
#include <csignal>

#include "serial_port/serial_port_interface.hpp"

namespace RosSerialRobot
{
  struct SamplePacket
  {
    std::vector<uint8_t> headers; // Headers in HEX
    unsigned int id;              // Position of the id byte
    unsigned int length;          // Position of length byte
    unsigned int data;            // Start position of data byte
  };

  enum class SerialCommandError
  {
    NO_ERROR,
    READ_ERROR,
    WRITE_ERROR,
    NO_RESPONSE,
    WRONG_HEADER,
    WRONG_CHECKSUM,
  };

  class SerialRobot
  {
  public:
    SerialRobot(const unsigned int &timeout,
                const unsigned int &num_tries);
    SerialRobot();
    virtual ~SerialRobot();

    // Abstract functions for plugin implementations
    virtual bool configure() = 0;
    virtual uint8_t calcCheckSum(std::vector<uint8_t> &data) const = 0;
    virtual bool getJointsPosition(std::vector<double> &positions) = 0;
    virtual bool setJointsPosition(const std::vector<double> positions) = 0;

    void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port);
    int readResponse(std::vector<uint8_t> &response);
    int writeCommand(const std::vector<uint8_t> &command);

    // Utils function
    static void packFromFloats(
        const std::vector<float> &value_to_pack,
        std::vector<uint8_t> &packed_doubles);
    static void packFromDoubles(
        const std::vector<double> &value_to_pack,
        std::vector<uint8_t> &packed_doubles);
    static void unpackFloats(const std::vector<uint8_t>::iterator &it, float &output);
    static void floatToByte(float float_variable, uint8_t temp_bytes[]);
    static void doubleToByte(double double_variable, uint8_t temp_bytes[]);

  protected:
    std::shared_ptr<SerialPortInterface> serial_port_;
    unsigned int timeout_;
    unsigned int num_tries_;
    SamplePacket sample_packet_;
    std::mutex comms_mtx_;
  };

}

#endif