#include <iostream>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include <sensor_msgs/Temperature.h>
#include <manzanita_mk3_driver/BatteryPack.h>
#include <manzanita_mk3_driver/mk3_driver.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <river_ros_util/ros_util.h>

class MK3DriverNode {
private:
  int num_parallel_units_;
  int num_series_units_;
  int cells_per_unit_;

  std::string frame_id_;
  double max_cell_voltage_;
  double max_total_voltage_;
  double warn_total_voltage_;
  double critical_total_voltage_;

private:
  manzanita_mk3_driver::MK3DriverPtr driver_;
  ros::Publisher batteryPublisher_;
  ros::Publisher temperaturePublisher_;
  ros::Timer updateTimer_;
  ros::Publisher diagnosticPublisher_;


protected:
  void update(const ros::TimerEvent& e){
      manzanita_mk3_driver::BatteryPack battery_msg;
      diagnostic_updater::DiagnosticStatusWrapper diagnostic_msg;
      diagnostic_msg.name = "Battery Pack";
      diagnostic_msg.hardware_id = frame_id_;

      battery_msg.total_voltage = 0;
      for(int i = 0; i<num_series_units_; ++i)
	battery_msg.total_voltage += driver_->getTotalVoltage(i*cells_per_unit_+1);
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

	  temp_msg.temperature = driver_->getHeatsinkTemperature(series_unit*cells_per_unit_+parallel_unit*num_series_units_*cells_per_unit_+1);
	  temp_msg.header.frame_id = series_unit_frame_id.str()+"/heatsink";
	  temp_msg.header.stamp = ros::Time::now();
	  temperaturePublisher_.publish(temp_msg);
	  diagnostic_msg.add(series_unit_frame_id.str()+" Heatsink", temp_msg.temperature);


	  for(int cell = 0; cell<cells_per_unit_; ++cell){
	    std::ostringstream cell_frame_id;
	    cell_frame_id << series_unit_frame_id.str();
	    if(cells_per_unit_!=1)
	      cell_frame_id << "/cell" << cell;

	    cell_msg.cell_id = cell+series_unit*cells_per_unit_+parallel_unit*num_series_units_*cells_per_unit_+1;
	    cell_msg.voltage = driver_->getCellVoltage(cell_msg.cell_id);
	    cell_msg.frame_id = cell_frame_id.str();
	    battery_msg.cells.push_back(cell_msg);

	    temp_msg.temperature = driver_->getCellTemperature(cell_msg.cell_id);
	    temp_msg.header.frame_id = cell_frame_id.str();
	    temp_msg.header.stamp = ros::Time::now();
	    temperaturePublisher_.publish(temp_msg);
	    
	    diagnostic_msg.add(cell_frame_id.str()+" Temperature", temp_msg.temperature);
	    diagnostic_msg.add(cell_frame_id.str()+" Voltage", cell_msg.voltage);
	  }
	}
      }

      battery_msg.max_total_voltage = max_total_voltage_;
      battery_msg.warn_total_voltage = warn_total_voltage_;
      battery_msg.critical_total_voltage = critical_total_voltage_;
      battery_msg.max_cell_voltage = max_cell_voltage_;
      batteryPublisher_.publish(battery_msg);

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
  MK3DriverNode(ros::NodeHandle nh, ros::NodeHandle pnh) : frame_id_("0"){
    river_ros_util::get_param(pnh, frame_id_, "frame_id");
    river_ros_util::get_param(pnh, max_cell_voltage_, "max_cell_voltage");
    river_ros_util::get_param(pnh, max_total_voltage_, "max_total_voltage");
    river_ros_util::get_param(pnh, warn_total_voltage_, "warn_total_voltage");
    river_ros_util::get_param(pnh, critical_total_voltage_, "critical_total_voltage");
    define_and_get_param(pnh, double, update_rate, "update_rate", 0.25);

    river_ros_util::get_param(pnh, num_parallel_units_, "num_parallel_units");
    river_ros_util::get_param(pnh, num_series_units_, "num_series_units");
    river_ros_util::get_param(pnh, cells_per_unit_, "cells_per_unit");

    define_and_get_param(pnh, std::string, port, "port", "/dev/ttyUSB0");
    driver_ = manzanita_mk3_driver::MK3DriverPtr(new manzanita_mk3_driver::MK3Driver(port));


    batteryPublisher_ = nh.advertise<manzanita_mk3_driver::BatteryPack>("battery", 100);
    temperaturePublisher_ = nh.advertise<sensor_msgs::Temperature>("temperature", 100);
    updateTimer_ = nh.createTimer(ros::Duration(1/update_rate), &MK3DriverNode::update, this);
    diagnosticPublisher_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 100);
  }
  virtual ~MK3DriverNode(){}

};

int main (int argc, char *argv[]){
    ros::init(argc, argv, "mk3_driver");
    ros::NodeHandle n;
    ros::NodeHandle pn;

    MK3DriverNode node(n, pn);

    ros::spin();

    return 0;
}
