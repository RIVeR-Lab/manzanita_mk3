#include <iostream>
#include <string>
#include <sstream>
#include <manzanita_mk3_driver/mk3_driver.h>
#include <device_driver_base/driver_util.h>

namespace manzanita_mk3_driver{
  using namespace std;

  const int TIMEOUT = 100;

  const char* const _heatsink_temp = "t";
  const char _heatsink_temp_type = 'T';
  const char* const _cell_temp = "x";
  const char _cell_temp_type = 'X';
  const char* const _voltage = "v";
  const char _voltage_type = 'V';
  const char* const _tot_voltage = "q";
  const char _tot_voltage_type = 'Q';

  void MK3Driver::openDevice(){
    ROS_INFO_STREAM("Opending device '"<<port_<<"'");
    bms_port_.open(port_, B9600, 8, device_driver::serial_parity_none);
  }
  void MK3Driver::closeDevice(){
    ROS_INFO_STREAM("Closing device '"<<port_<<"'");
    bms_port_.close();
  }

  MK3Driver::MK3Driver(std::string port) : port_(port){
  }

  void MK3Driver::sendBMSInstruction(int address, const char* command){
    bms_port_.writef(TIMEOUT, "%d%s.\n\r", address, command);
  }
  void MK3Driver::sendBMSInstruction(int address, const char* command, int value){
    bms_port_.writef(TIMEOUT, "%d%s.%d\n\r", address, command, value);
  }
  void MK3Driver::sendBMSInstruction(int address, const char* command, double value){
    bms_port_.writef(TIMEOUT, "%d%s.%f\n\r", address, command, value);
  }


#define RESPONSE_BUFFER_SIZE 40
  void MK3Driver::readBMSResponse(int address, char type, char* result){
    char buf[RESPONSE_BUFFER_SIZE+1];
    int response_address = -1;
    char response_type;
    while(response_address!=address || response_type!=type){
      bms_port_.read_until(buf, RESPONSE_BUFFER_SIZE, '\r', TIMEOUT);
      buf[strlen(buf)-1] = '\0';//remove \n that was before \r
      if(sscanf(buf, "%d%c %s", &response_address, &response_type, result)!=3)
	response_address = -1;
    }
  }

  float MK3Driver::getCellVoltage(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _voltage);
    readBMSResponse(node, _voltage_type, value);

    float voltage;
    if(sscanf(value, "%fV", &voltage)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get voltage with bad value: %s", value);
    return voltage;
  }
  float MK3Driver::getTotalVoltage(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _tot_voltage);
    readBMSResponse(node, _tot_voltage_type, value);

    float voltage;
    if(sscanf(value, "%fV", &voltage)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get total voltage with bad value: %s", value);
    return voltage;
  }


  int MK3Driver::getHeatsinkTemperature(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _heatsink_temp);
    readBMSResponse(node, _heatsink_temp_type, value);

    int temp;
    if(sscanf(value, "%dF", &temp)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get heatsink temp with bad value: %s", value);
    return temp;
  }
  int MK3Driver::getCellTemperature(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _cell_temp);
    readBMSResponse(node, _cell_temp_type, value);

    int temp;
    if(sscanf(value, "%dF", &temp)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get cell temp with bad value: %s", value);
    return temp;
  }

}
