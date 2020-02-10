#include "ros/ros.h"
#include <iostream>
// #include "std_msgs/String.h"
// #include "sensor_msgs/Imu.h"
#include "rosutils/sonarmsg.h"
#include "roverbot/MB1242.h"
#include <math.h>       /* fabs */
#include <memory>

#include <dynamic_reconfigure/server.h>
#include <roverbot/RoverbotConfig.h>

// #include "beginner_tutorials/MPU6050.h"


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
// }

uint16_t dist_cm;
// int_t freq;
std::unique_ptr<ros::Rate> loop_rate_ptr;

void dynamic_cfg_callback(roverbot::RoverbotConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d %s", 
            config.sonar_freq, 
            config.imu_freq, 
            config.cam_freq, 
            config.cam_on_off?"True":"False"
            );

      if (config.sonar_freq<0){
        loop_rate_ptr.reset(new ros::Rate(1/std::fabs(config.sonar_freq)));
      }
      else{
        loop_rate_ptr.reset(new ros::Rate(config.sonar_freq));
      }
      
      
      

}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "roverbot_sonar"); // node name has to be unique

  ros::NodeHandle nh("roverbot");  // the striing here is used for initial namespace of messages
  nh.setParam("sonarfreq", 1);

  ros::Publisher sonar_pub = nh.advertise<rosutils::sonarmsg>("sonar", 10);

  

  dynamic_reconfigure::Server<roverbot::RoverbotConfig> server;
  dynamic_reconfigure::Server<roverbot::RoverbotConfig>::CallbackType f;
  f = boost::bind(&dynamic_cfg_callback, _1, _2);
  server.setCallback(f);

  MB1242 SonarSensor;
  SonarSensor.begin();

  loop_rate_ptr.reset( new ros::Rate(1) );
  
  ros::Duration(0.5).sleep();

  while (ros::ok())
  {

    rosutils::sonarmsg sonar;
    
    SonarSensor.requestDistance();
    ros::Duration(0.1).sleep();
    sonar.dist_cm = SonarSensor.getDistance();

    sonar_pub.publish(sonar);

    ros::spinOnce();

    loop_rate_ptr->sleep();


  }


  return 0;
}
