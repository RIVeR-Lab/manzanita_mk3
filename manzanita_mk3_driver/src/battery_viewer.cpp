#include <iostream>
#include <string>
#include <sstream>

#include "ros/ros.h"
#include <sensor_msgs/Temperature.h>
#include <manzanita_mk3_driver/BatteryPack.h>
#include <boost/foreach.hpp>

void battery_callback(manzanita_mk3_driver::BatteryPackConstPtr battery){
  printf("Battery Voltage: %fV\n", battery->total_voltage);
  BOOST_FOREACH(manzanita_mk3_driver::BatteryCell cell, battery->cells){
    printf("\t%s: %fV\n", cell.frame_id.c_str(), cell.voltage);
  }
  printf("\n");
}

int main (int argc, char *argv[]){
    ros::init(argc, argv, "battery_viewer");
    ros::NodeHandle n;

    ros::Subscriber batteryPublisher = n.subscribe("battery", 6, battery_callback);
    //ros::Subscriber temperaturePublisher = n.subscribe("temperature",15);

    ros::spin();

    return(0);
}
