#ifndef SERIAL_PORT__SERIAL_PORT_INTERFACE_HPP_
#define SERIAL_PORT__SERIAL_PORT_INTERFACE_HPP_

#include <vector>

class SerialPortInterface
{
public:
  SerialPortInterface(){};
  virtual ~SerialPortInterface(){};

  virtual void openPort() = 0;
  virtual void closePort() = 0;

  virtual void write(const std::vector<uint8_t> &data) = 0;
  virtual void wait() = 0;

  virtual unsigned int read(std::vector<uint8_t> &data) = 0;

  virtual int available() = 0;
  
  virtual void setReadDataStream(const std::vector<uint8_t> &data) = 0;
  virtual void getWriteDataStream(std::vector<uint8_t> &data) = 0 ;
};

#endif