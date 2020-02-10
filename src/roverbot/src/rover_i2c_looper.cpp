#include "ros/ros.h"
#include <iostream>
#include <ros/console.h>
#include <math.h>       /* fabs */
#include <memory>
#include <mutex>
// #include "std_msgs/String.h"
// #include "sensor_msgs/Imu.h"
#include "rosutils/RawImu.h"
#include "roverbot/MPU6050.h"

#include "rosutils/sonarmsg.h"
#include "roverbot/MB1242.h"

#include "rosutils/batterymsg.h"
#include "roverbot/INA219.h"

#include "geometry_msgs/Twist.h"
#include "rosutils/roverpwmcontrol.h"


#include "roverbot/I2Cdev.h"

// #include "beginner_tutorials/MPU6050.h"

#include <dynamic_reconfigure/server.h>
#include <roverbot/RoverbotConfig.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
// }

static uint8_t PWMI2CADDRESS = 8; 
std::shared_ptr<I2Cdev> i2cCtrl = I2Cdev::getInstance();

std::unique_ptr<ros::Rate> loop_rate_ptr;
Adafruit_INA219 ina219;

float sonar_freq=1; 
float imu_freq=1 ;
float battery_freq=1; 
float cam_freq=1;
bool cam_on_off=false;


void dynamic_cfg_callback(roverbot::RoverbotConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d %d %s", 
            config.sonar_freq, 
            config.imu_freq, 
            config.battery_freq, 
            config.cam_freq, 
            config.cam_on_off?"True":"False"
            );

      cam_on_off = config.cam_on_off;

      sonar_freq = config.sonar_freq;
      if (sonar_freq<0){
        sonar_freq = 1/std::fabs(sonar_freq);
        // loop_rate_ptr.reset(new ros::Rate(1/std::fabs(config.imu_freq)));
      }
      
      imu_freq = config.imu_freq;
      if (imu_freq<0){
        imu_freq = 1/std::fabs(imu_freq);
        // loop_rate_ptr.reset(new ros::Rate(1/std::fabs(config.imu_freq)));
      }
      
      battery_freq = config.battery_freq;
      if (battery_freq<0){
        battery_freq = 1/std::fabs(battery_freq);
        // loop_rate_ptr.reset(new ros::Rate(1/std::fabs(config.imu_freq)));
      }
      
      cam_freq = config.cam_freq;
      if (cam_freq<0){
        cam_freq = 1/std::fabs(cam_freq);
        // loop_rate_ptr.reset(new ros::Rate(1/std::fabs(config.imu_freq)));
      }
      


}

void pwm_main_Callback(const rosutils::roverpwmcontrol::ConstPtr& msg)
{ 
  uint8_t data[4];
  // std::lock_guard<std::mutex> lockg(pwmmutex);
  data[0] = msg->durn;
  data[1] = msg->Lpwm;
  data[2] = msg->Rpwm;
  data[3] = msg->dirn;
  
  ROS_INFO("writing pwm");
  i2cCtrl->writeBytes(PWMI2CADDRESS, 0, 4, data);
    ROS_INFO("main pwm setting: L:[%d], R:[%d], dirn:[%d], duration:[%d]", msg->Lpwm, msg->Rpwm, msg->dirn, msg->durn);
}


void pwm_twist_Callback(const geometry_msgs::Twist::ConstPtr& msg)

{
  
     // ROS_DEBUG("twist pwm setting: %f %f", msg->linear.x, msg->angular.z);

    int Lpwm,Rpwm,dirn,durn;
  uint8_t data[4];
  
  // std::lock_guard<std::mutex> lockg(pwmmutex);
  durn = 250;

  if (msg->linear.x>0 && msg->angular.z==0){
    Lpwm = msg->linear.x;
    Rpwm = msg->linear.x;
    dirn = 22;
  }
  else if(msg->linear.x<0 && msg->angular.z==0){
    Lpwm = -msg->linear.x;
    Rpwm = -msg->linear.x;
    dirn = 11;
  }
  else if (msg->linear.x==0 && msg->angular.z>0){
    Lpwm = msg->angular.z;
    Rpwm = msg->angular.z;
    dirn = 12;
  }
  else if(msg->linear.x==0 && msg->angular.z<0){
    Lpwm = -msg->angular.z;
    Rpwm = -msg->angular.z;
    dirn = 21;
  }
  else{
    // ROS_INFO(" No twist pwm input");
    return;
  }
  data[0] = durn;
  data[1] = Lpwm;
  data[2] = Rpwm;
  data[3] = dirn;

  ROS_INFO("writing pwm");
  i2cCtrl->writeBytes(PWMI2CADDRESS, 0, 4, data);

    ROS_INFO("twist setting: L:[%d], R:[%d], dirn:[%d], duration:[%d]", Lpwm, Rpwm, dirn, durn);

    // ros::Duration(0,durn*1000).sleep();  // durn*1000 in nano seconds

}

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
  
  ros::init(argc, argv, "roverbot_imu");

  ros::NodeHandle nh("roverbot");


  ros::Publisher imu_pub = nh.advertise<rosutils::RawImu>("imu", 5);
  ros::Publisher sonar_pub = nh.advertise<rosutils::sonarmsg>("sonar", 10);
  ros::Publisher battery_pub = nh.advertise<rosutils::batterymsg>("battery", 2);

  ros::Subscriber subdirectctrl = nh.subscribe("lrdd", 5, pwm_main_Callback);
  ros::Subscriber subtwistctrl = nh.subscribe("key_vel", 5, pwm_twist_Callback);

  dynamic_reconfigure::Server<roverbot::RoverbotConfig> server;
  dynamic_reconfigure::Server<roverbot::RoverbotConfig>::CallbackType f;
  f = boost::bind(&dynamic_cfg_callback, _1, _2);
  server.setCallback(f);

  float loopfreq = 1000;
  loop_rate_ptr.reset( new ros::Rate(loopfreq) );

  MPU6050 accelgyro;
  accelgyro.initialize();

  MB1242 SonarSensor;
  SonarSensor.begin();

  ina219.begin();
  ina219.setCalibration_32V_2A();

  double AccGyro[7];

  int imucnt = 0;
  int sonarcnt = 0;
  int battcnt = 0;


  while (ros::ok())
  {

    if(imucnt > loopfreq/imu_freq){
      // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      accelgyro.getScaledaccgyro_timestamped(AccGyro);
      // std::cout << ax << " " << ay << " " << az << " " << gx << " " << gy << " " << gz << std::endl;

      rosutils::RawImu imu;
      imu.linear_acceleration.x = AccGyro[1];
      imu.linear_acceleration.y = AccGyro[2];
      imu.linear_acceleration.z = AccGyro[3];

      imu.angular_velocity.x = AccGyro[4];
      imu.angular_velocity.y = AccGyro[5];
      imu.angular_velocity.z = AccGyro[6];

      imu_pub.publish(imu);
      ROS_INFO("IMU %d %f %f %f",imucnt,loopfreq,imu_freq,(loopfreq/imu_freq));
      imucnt=0;
    }

    if(sonarcnt > loopfreq/sonar_freq){
      rosutils::sonarmsg sonar;
      
      SonarSensor.requestDistance();
      ros::Duration(0.08).sleep();
      sonar.dist_cm = SonarSensor.getDistance();

      sonar_pub.publish(sonar);

      sonarcnt=0;

    }

    if(battcnt > loopfreq/battery_freq){

      rosutils::batterymsg battery;
      setBatteryMeasurements(battery);
      battery_pub.publish(battery);

      battcnt=0;
    }


    ros::spinOnce();

    loop_rate_ptr->sleep();

    imucnt=imucnt+1;
    sonarcnt=sonarcnt+1;
    battcnt=battcnt+1;
  }


  return 0;
}
