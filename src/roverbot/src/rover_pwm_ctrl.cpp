#include "ros/ros.h"
#include <ros/console.h>
#include <iostream>
// #include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "rosutils/roverpwmcontrol.h"
#include <mutex>
#include <memory>
#include "roverbot/I2Cdev.h"

static uint8_t PWMI2CADDRESS = 8; 

std::mutex pwmmutex;
std::shared_ptr<I2Cdev> i2cCtrl = I2Cdev::getInstance();

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

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

int main(int argc, char **argv)
{ 

  ros::init(argc, argv, "roverbot_motion");

  ros::NodeHandle nh("roverbot");

  ros::Subscriber subdirectctrl = nh.subscribe("lrdd", 5, pwm_main_Callback);
  ros::Subscriber subtwistctrl = nh.subscribe("key_vel", 5, pwm_twist_Callback);

  ROS_INFO("spinnign now");
  std::cout<<"Spin cout now"<<std::endl;

  ros::spin();

  return 0;
}
