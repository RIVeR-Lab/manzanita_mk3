#ifndef MANZANITA_MK3_DRIVER_MK3_DRIVER_H_
#define MANZANITA_MK3_DRIVER_MK3_DRIVER_H_

#include "ros/ros.h"
#include <device_driver_base/serial_port.h>

namespace manzanita_mk3_driver{

class MK3Driver {
private:
  std::string port_;

private:
  device_driver::DriverSerialPort bms_port_;

protected:
  void openDevice();
  void closeDevice();

public:
  MK3Driver(std::string port);
  virtual ~MK3Driver(){}

private:
  void sendBMSInstruction(int address, const char* command);
  void sendBMSInstruction(int address, const char* command, int value);
  void sendBMSInstruction(int address, const char* command, double value);
  void readBMSResponse(int address, char type, char* result);

public:
  float getCellVoltage(int node);
  float getTotalVoltage(int node);
  int getHeatsinkTemperature(int node);
  int getCellTemperature(int node);

};

typedef boost::shared_ptr<MK3Driver> MK3DriverPtr;

}

#endif
