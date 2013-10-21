#include <iostream>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include <sensor_msgs/Temperature.h>
#include <manzanita_mk3_driver/BatteryPack.h>
#include <device_driver_base/serial_port.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <device_driver_base/reconfigurable_device_driver.h>
#include <manzanita_mk3_driver/MK3Config.h>

using namespace std;
using namespace device_driver;

const int TIMEOUT = 100;

const char* const _heatsink_temp = "t";
const char _heatsink_temp_type = 'T';
const char* const _cell_temp = "x";
const char _cell_temp_type = 'X';
const char* const _voltage = "v";
const char _voltage_type = 'V';
const char* const _tot_voltage = "q";
const char _tot_voltage_type = 'Q';




class MK3Driver : public ReconfigurableDeviceDriver<manzanita_mk3_driver::MK3Config>{
private:
  std::string port_;
  int num_parallel_units_;
  int num_series_units_;
  int cells_per_unit_;
  std::string frame_id_;
  double max_cell_voltage_;
  double max_total_voltage_;
  double warn_total_voltage_;
  double critical_total_voltage_;

private:
  device_driver::DriverSerialPort bms_port_;
  device_driver::ReconfigurableAdvertisePtr batteryPublisher_;
  device_driver::ReconfigurableAdvertisePtr temperaturePublisher_;
  device_driver::ReconfigurableTimerPtr updateTimer_;
  ros::Publisher diagnosticPublisher_;



protected:
  void openDevice(){
    ROS_INFO_STREAM("Opending device '"<<port_<<"'");
    bms_port_.open(port_, B9600, 8, serial_parity_none);
  }
  void closeDevice(){
    ROS_INFO_STREAM("Closing device '"<<port_<<"'");
    bms_port_.close();
  }
  void update(const ros::TimerEvent& e){
      manzanita_mk3_driver::BatteryPack battery_msg;
      diagnostic_updater::DiagnosticStatusWrapper diagnostic_msg;
      diagnostic_msg.name = "Battery Pack";
      diagnostic_msg.hardware_id = frame_id_;

      battery_msg.total_voltage = 0;
      for(int i = 0; i<num_series_units_; ++i)
	battery_msg.total_voltage+=getTotalVoltage(i*cells_per_unit_+1);
      diagnostic_msg.add("Total Voltage", battery_msg.total_voltage);

      manzanita_mk3_driver::BatteryCell cell_msg;
      sensor_msgs::Temperature temp_msg;
      temp_msg.variance = 0;
      for(int parallel_unit = 0; parallel_unit<num_parallel_units_; ++parallel_unit){
	std::ostringstream parallel_unit_frame_id;
	parallel_unit_frame_id << frame_id_;
	if(num_parallel_units_!=1)
	  parallel_unit_frame_id << "/parallel" << parallel_unit;

	for(int series_unit = 0; series_unit<num_series_units_; ++series_unit){
	  std::ostringstream series_unit_frame_id;
	  series_unit_frame_id << parallel_unit_frame_id.str();
	  if(num_series_units_!=1)
	    series_unit_frame_id << "/series" << series_unit;

	  temp_msg.temperature = getHeatsinkTemperature(series_unit*cells_per_unit_+parallel_unit*num_series_units_*cells_per_unit_+1);
	  temp_msg.header.frame_id = series_unit_frame_id.str()+"/heatsink";
	  temp_msg.header.stamp = ros::Time::now();
	  temperaturePublisher_->publish(temp_msg);
	  diagnostic_msg.add(series_unit_frame_id.str()+" Heatsink", temp_msg.temperature);


	  for(int cell = 0; cell<cells_per_unit_; ++cell){
	    std::ostringstream cell_frame_id;
	    cell_frame_id << series_unit_frame_id.str();
	    if(cells_per_unit_!=1)
	      cell_frame_id << "/cell" << cell;

	    cell_msg.cell_id = cell+series_unit*cells_per_unit_+parallel_unit*num_series_units_*cells_per_unit_+1;
	    cell_msg.voltage = getCellVoltage(cell_msg.cell_id);
	    cell_msg.frame_id = cell_frame_id.str();
	    battery_msg.cells.push_back(cell_msg);

	    temp_msg.temperature = getCellTemperature(cell_msg.cell_id);
	    temp_msg.header.frame_id = cell_frame_id.str();
	    temp_msg.header.stamp = ros::Time::now();
	    temperaturePublisher_->publish(temp_msg);
	    
	    diagnostic_msg.add(cell_frame_id.str()+" Temperature", temp_msg.temperature);
	    diagnostic_msg.add(cell_frame_id.str()+" Voltage", cell_msg.voltage);
	  }
	}
      }

      battery_msg.max_total_voltage = max_total_voltage_;
      battery_msg.warn_total_voltage = warn_total_voltage_;
      battery_msg.critical_total_voltage = critical_total_voltage_;
      battery_msg.max_cell_voltage = max_cell_voltage_;
      batteryPublisher_->publish(battery_msg);

      if(battery_msg.total_voltage>warn_total_voltage_)
	diagnostic_msg.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Battery OK (%fV)", battery_msg.total_voltage);
      else if(battery_msg.total_voltage>critical_total_voltage_)
	diagnostic_msg.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Battery Getting Low!!! (%fV)", battery_msg.total_voltage);
      else
	diagnostic_msg.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery VERY LOW (%fV)", battery_msg.total_voltage);

      diagnostic_msgs::DiagnosticArray diagnostics_msg;
      diagnostics_msg.status.push_back(diagnostic_msg);
      diagnostics_msg.header.stamp = ros::Time::now();
      diagnosticPublisher_.publish(diagnostics_msg);
  }

public:
  MK3Driver() : port_("/dev/ttyUSB0"), num_parallel_units_(1), num_series_units_(1), cells_per_unit_(6), frame_id_("0"){
	batteryPublisher_ = addReconfigurableAdvertise<manzanita_mk3_driver::BatteryPack>(device_driver_state::RUNNING, "/battery", 100);
    temperaturePublisher_ = addReconfigurableAdvertise<sensor_msgs::Temperature>(device_driver_state::RUNNING, "/temperature", 100);
    updateTimer_ = addReconfigurableTimer(device_driver_state::RUNNING, ros::Duration(4), &MK3Driver::update, this);

    diagnosticPublisher_ = node_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);

    addDriverStateFunctions(device_driver_state::OPEN, &MK3Driver::openDevice, &MK3Driver::closeDevice, this);
  }
  virtual ~MK3Driver(){}

  virtual void reconfigureStopped(manzanita_mk3_driver::MK3Config& config){
	  port_ = config.port;
  }
  virtual void reconfigureOpen(manzanita_mk3_driver::MK3Config& config){
	  num_parallel_units_ = config.num_parallel_units;
	  num_series_units_ = config.num_series_units;
	  cells_per_unit_ = config.cells_per_unit;

	  updateTimer_->setPeriod(ros::Duration(1/config.update_rate));

	  batteryPublisher_->setTopic(config.battery_topic);
	  temperaturePublisher_->setTopic(config.temperature_topic);
  }
  virtual void reconfigureRunning(manzanita_mk3_driver::MK3Config& config){
    frame_id_ = config.frame_id;
    max_cell_voltage_ = config.max_cell_voltage;
    max_total_voltage_ = config.max_total_voltage;
    warn_total_voltage_ = config.warn_total_voltage;
    critical_total_voltage_ = config.critical_total_voltage;
  }

private:
  void sendBMSInstruction(int address, const char* command){
    bms_port_.writef(TIMEOUT, "%d%s.\n\r", address, command);
  }
  void sendBMSInstruction(int address, const char* command, int value){
    bms_port_.writef(TIMEOUT, "%d%s.%d\n\r", address, command, value);
  }
  void sendBMSInstruction(int address, const char* command, double value){
    bms_port_.writef(TIMEOUT, "%d%s.%f\n\r", address, command, value);
  }


#define RESPONSE_BUFFER_SIZE 40
  void readBMSResponse(int address, char type, char* result){
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

public:
  float getCellVoltage(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _voltage);
    readBMSResponse(node, _voltage_type, value);

    float voltage;
    if(sscanf(value, "%fV", &voltage)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get voltage with bad value: %s", value);
    return voltage;
  }
  float getTotalVoltage(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _tot_voltage);
    readBMSResponse(node, _tot_voltage_type, value);

    float voltage;
    if(sscanf(value, "%fV", &voltage)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get total voltage with bad value: %s", value);
    return voltage;
  }


  int getHeatsinkTemperature(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _heatsink_temp);
    readBMSResponse(node, _heatsink_temp_type, value);

    int temp;
    if(sscanf(value, "%dF", &temp)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get heatsink temp with bad value: %s", value);
    return temp;
  }
  int getCellTemperature(int node){
    char value[RESPONSE_BUFFER_SIZE+1];

    sendBMSInstruction(node, _cell_temp);
    readBMSResponse(node, _cell_temp_type, value);

    int temp;
    if(sscanf(value, "%dF", &temp)!=1)
      DRIVER_EXCEPT(device_driver::CorruptDataException, "Got response to get cell temp with bad value: %s", value);
    return temp;
  }

};

int main (int argc, char *argv[]){
    ros::init(argc, argv, "mk3_driver");
    ros::NodeHandle n;

    MK3Driver driver;

    driver.spin();

    return(0);
}
