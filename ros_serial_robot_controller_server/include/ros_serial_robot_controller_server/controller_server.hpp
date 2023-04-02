#ifndef ROS_SERIAL_ROBOT_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_
#define ROS_SERIAL_ROBOT_CONTROLLER_SERVER__CONTROLLER_SERVER_HPP_

#include <iostream>
#include <thread>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include "ros_serial_robot_driver/serial_robot.hpp"
#include "serial_port/serial_port_interface.hpp"

namespace RosSerialRobot
{
  class ControllerServer
  {
  public:
    ControllerServer();
    virtual ~ControllerServer();

    bool configure();

  protected:
    boost::shared_ptr<RosSerialRobot::SerialRobot> robot_driver_;
    std::shared_ptr<pluginlib::ClassLoader<RosSerialRobot::SerialRobot>> plugin_loader_;

    std::shared_ptr<SerialPortInterface> serial_port_;
  };
} // namespace RosSerialRobot

#endif