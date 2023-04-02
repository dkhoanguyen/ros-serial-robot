#include <pluginlib/class_loader.h>
#include "ros_serial_robot_driver/serial_robot.hpp"

int main()
{
  pluginlib::ClassLoader<RosSerialRobot::SerialRobot> poly_loader(
      "ros_serial_robot_driver", "RosSerialRobot::SerialRobot");

  try
  {
    boost::shared_ptr<RosSerialRobot::SerialRobot> triangle = poly_loader.createInstance("HansCuteRobot::HansCuteDriver");
    triangle->configure();
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  return 0;
}