#ifndef ROS_SERIAL_ROBOT__SERIAL_ROBOT_HPP_
#define ROS_SERIAL_ROBOT__SERIAL_ROBOT_HPP_

#include <thread>
#include <vector>
#include "serial_port/serial_port_interface.hpp"
#include "serial_robot_interface.hpp"

class SerialRobot : public SerialRobotInterface
{
  SerialRobot();
  virtual ~SerialRobot();

  bool setup();
  int open();
  int close();
  void setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port);

  int readResponse(std::vector<uint8_t> &response);
  int writeCommand(const std::vector<uint8_t> &command);

  virtual uint8_t calcCheckSum(std::vector<uint8_t> &data) const = 0;

protected:
  std::shared_ptr<SerialPortInterface> serial_port_;
};

#endif