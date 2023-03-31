#include "ros_serial_robot_driver/serial_robot.hpp"

SerialRobot::SerialRobot()
{
}

SerialRobot::~SerialRobot()
{
}

bool SerialRobot::setup()
{
}

int SerialRobot::open()
{

}

int SerialRobot::close()
{

}

void SerialRobot::setSerialPort(const std::shared_ptr<SerialPortInterface> &serial_port)
{
  serial_port_ = serial_port;
}