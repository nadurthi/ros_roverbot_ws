#include "ros/ros.h"
#include <iostream>
// #include "std_msgs/String.h"
// #include "sensor_msgs/Imu.h"
#include "rosutils/batterymsg.h"
#include "roverbot/INA219.h"
#include <memory>

// #include "beginner_tutorials/MPU6050.h"

Adafruit_INA219 ina219;

void setBatteryMeasurements(rosutils::batterymsg& battery){
  battery.shuntvoltage = 0;
  battery.busvoltage = 0;
  battery.current_mA = 0;
  battery.loadvoltage = 0;
  battery.power_mW = 0;

  battery.shuntvoltage = ina219.getShuntVoltage_mV();
  battery.busvoltage = ina219.getBusVoltage_V();
  battery.current_mA = ina219.getCurrent_mA();
  battery.power_mW = ina219.getPower_mW();
  battery.loadvoltage = battery.busvoltage + (battery.shuntvoltage / 1000);

  ROS_INFO("Sending battery %f, %f, %f, %f",battery.shuntvoltage,battery.busvoltage,battery.current_mA,battery.loadvoltage);

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rover_battery_publish_node");

  ros::NodeHandle nh("roverbot");
  nh.setParam("batterySensfreq", 1);

  ros::Publisher battery_pub = nh.advertise<rosutils::batterymsg>("rover_battery", 100);

  std::unique_ptr<ros::Rate> loop_rate_ptr( new ros::Rate(1) );
  int  batterySensfreq=1,prevbatterySensfreq=1;

  
  ina219.begin();
  ina219.setCalibration_32V_2A();

  ros::Duration(0.5).sleep();
  rosutils::batterymsg battery;

  // int count = 0;
  while (ros::ok())
  {

    setBatteryMeasurements(battery);
    battery_pub.publish(battery);

    

    ros::spinOnce();

    loop_rate_ptr->sleep();

    nh.getParam("batterySensfreq", batterySensfreq);
    
    if (batterySensfreq!=prevbatterySensfreq){
      loop_rate_ptr.reset(new ros::Rate(batterySensfreq));
      prevbatterySensfreq = batterySensfreq;
    }

    // ++count;
  }


  return 0;
}
