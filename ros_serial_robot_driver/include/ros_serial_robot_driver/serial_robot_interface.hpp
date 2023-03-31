#ifndef ROS_SERIAL_ROBOT__SERIAL_ROBOT_INTERFACE_HPP_
#define ROS_SERIAL_ROBOT__SERIAL_ROBOT_INTERFACE_HPP_

#include <string>
#include <vector>
#include "serial_port/serial_port_interface.hpp"

class SerialRobotInterface
{
public:
  SerialRobotInterface() {};
  virtual ~SerialRobotInterface() {};

  virtual bool setup() = 0;
  virtual int open() = 0;
  virtual int close() = 0;

  virtual void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port) = 0;
  
  virtual int readResponse(std::vector<uint8_t> &response) = 0;
  virtual int writeCommand(const std::vector<uint8_t> &command) = 0;

  virtual uint8_t calcCheckSum(std::vector<uint8_t> &data) const = 0;

};

#endif