#include "ros/ros.h"
#include <iostream>
// #include "std_msgs/String.h"
// #include "sensor_msgs/Imu.h"
#include "rosutils/sonarmsg.h"
#include "roverbot/MB1242.h"

#include <memory>

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




int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rover_sonar_publish_node");

  ros::NodeHandle nh("roverbot");
  nh.setParam("sonarfreq", 1);

  ros::Publisher sonar_pub = nh.advertise<rosutils::sonarmsg>("rover_sonar", 10);

  std::unique_ptr<ros::Rate> loop_rate_ptr( new ros::Rate(1) );
  int sonarfreq=1,prevsonarfreq=1;

  MB1242 SonarSensor;
  SonarSensor.begin();

  ros::Duration(0.5).sleep();

  while (ros::ok())
  {

    rosutils::sonarmsg sonar;
    
    SonarSensor.requestDistance();
    ros::Duration(0.1).sleep();
    sonar.dist_cm = SonarSensor.getDistance();

    sonar_pub.publish(sonar);

    ROS_INFO("Sending Sonar %d",sonar.dist_cm);

    ros::spinOnce();

    loop_rate_ptr->sleep();

    nh.getParam("sonarfreq", sonarfreq);
    if (sonarfreq>10)
      sonarfreq=10;

    if (sonarfreq!=prevsonarfreq){
      loop_rate_ptr.reset(new ros::Rate(sonarfreq));
      prevsonarfreq = sonarfreq;
    }

    // ++count;
  }


  return 0;
}
