#ifndef SERIAL_PORT__DUMMY_SERIAL_PORT_HPP_
#define SERIAL_PORT__DUMMY_SERIAL_PORT_HPP_


#include <string>
#include <iostream>

#include "serial_port_interface.hpp"

class DummySerialPort : public SerialPortInterface
{
public:
  DummySerialPort(const std::string& port, const unsigned int& baud_rate);
  ~DummySerialPort();

  void openPort();
  void closePort();

  void write(const std::vector<uint8_t> &data);
  void wait();

  unsigned int read(std::vector<uint8_t> &data);
  int available();

  void setReadDataStream(const std::vector<uint8_t> &data);
  void getWriteDataStream(std::vector<uint8_t> &data);

private:
  std::string port_;
  unsigned int baud_rate_;
  std::vector<uint8_t> write_data_stream_;
  std::vector<uint8_t> read_data_stream_;
};

#endif