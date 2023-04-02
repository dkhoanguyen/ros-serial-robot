#ifndef HANS_CUTE_DRIVER__HANS_CUTE_DRIVER_HPP_
#define HANS_CUTE_DRIVER__HANS_CUTE_DRIVER_HPP_

#include "hans_cute_driver/hans_cute_const.hpp"
#include "hans_cute_driver/hans_cute_datatype.hpp"
#include "ros_serial_robot_driver/serial_robot.hpp"

namespace HansCuteRobot
{
  class HansCuteDriver : public RosSerialRobot::SerialRobot
  {
  public:
    HansCuteDriver();
    virtual ~HansCuteDriver();

    bool configure();

    uint8_t calcCheckSum(std::vector<uint8_t> &data) const;
    bool getJointsPosition(std::vector<double> &positions);
    bool setJointsPosition(const std::vector<double> positions);

  protected:
    bool read(const uint8_t &id, const uint8_t &address, const uint8_t &size, std::vector<uint8_t> &returned_data,
              unsigned long &timestamp);
    bool write(const uint8_t &id, const uint8_t &address, const std::vector<uint8_t> &data,
               std::vector<uint8_t> &returned_data, unsigned long &timestamp);
    bool syncWrite(const uint8_t &address, const std::vector<std::vector<uint8_t>> &data);

    bool ping(const uint8_t &id, std::vector<uint8_t> &returned_data);

    //====================================================================//
    // These function modify EEPROM data which persists after power cycle //
    //====================================================================//

    // These can be left until the end as they are relatively unimportant
    bool setID(const uint8_t &old_id, const uint8_t &new_id);
    bool setBaudrate(const uint8_t &servo_id, const long &baudrate);

    bool setReturnDelayTime(const uint8_t &servo_id, const unsigned int &delay_time);
    bool setAngleLimits(const uint8_t &servo_id, const unsigned int &min_limit, const unsigned int &max_limit);
    bool setVoltageLimits(const uint8_t &servo_id, const VoltageLimits &voltage_limits);
    bool setMaxTorque(const uint8_t &servo_id, const unsigned int &max_torque);

    //===============================================================//
    // These functions can send multiple commands to a single servo  //
    //===============================================================//
    bool setTorqueEnable(const uint8_t &servo_id, const bool &enabled);
    bool setComplianceMargin();
    bool setComplianceSlope();

    bool setDGain();
    bool setIGain();
    bool setPGain();

    bool setAcceleration(const uint8_t &servo_id, const unsigned int &acceleration);
    bool setPosition(const uint8_t &servo_id, const unsigned int &position);
    bool setSpeed(const uint8_t &servo_id, const unsigned int &speed);

    bool setTorqueLimit(const uint8_t &servo_id, const unsigned int &torque_limit);
    bool setGoalTorque();

    bool setPositionAndSpeed();

    //===============================================================//
    // These functions can send multiple commands to multiple servos //
    //===============================================================//

    bool setMultiTorqueEnabled();

    // Range is between 0 -> 255
    bool setMultiComplianceMargin();
    bool setMultiComplianceSlope();

    // Position value ranges from 0 -> 4095 (0xFFF), unit is 0.088 degree
    bool setMultiPosition(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &positions);
    bool setMultiSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &speeds);

    bool setMultiTorqueLimit(const std::vector<unsigned int> &servo_ids,
                             const std::vector<double> &torque_limits);
    bool setMultiPositionAndSpeed(const std::vector<unsigned int> &servo_ids, const std::vector<unsigned int> &positions,
                                  const std::vector<unsigned int> &speeds);

    //===============================//
    // Servo status access functions //
    //===============================//

    bool getModelNumber(const int &servo_id, unsigned int &model_number);
    bool getFirmwareVersion();
    bool getReturnDelayTime();

    bool getMaxTorque(const int &servo_id, unsigned int &torque);
    bool getTorqueEnabled(const int &servo_id, bool &enabled);
    bool getAngleLimits(const int &servo_id, HansCuteRobot::AngleLimits &angle_limit);

    bool getVoltageLimits();
    bool getPosition(const int &servo_id, unsigned int &position);
    bool getSpeed(const int &servo_id, unsigned int &speed);
    bool getAcceleration(const int &servo_id, unsigned int &acceleration);

    bool getVoltage();
    bool getCurrent();
    bool getLock(const int &servo_id, bool &lock);

    bool getFeedback(const int &servo_id, ServoFeedback &feedback);
  };
}
#endif