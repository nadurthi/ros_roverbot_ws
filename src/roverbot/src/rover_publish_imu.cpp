#include "ros/ros.h"
#include <iostream>
// #include "std_msgs/String.h"
// #include "sensor_msgs/Imu.h"
#include "rosutils/RawImu.h"
#include "roverbot/MPU6050.h"
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


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "rover_imu_publish_node");

  ros::NodeHandle nh("roverbot");
  nh.setParam("imufreq", 1);

  ros::Publisher imu_pub = nh.advertise<rosutils::RawImu>("rover_imu", 100);

  // ros::Rate loop_rate(1);
  std::unique_ptr<ros::Rate> loop_rate_ptr( new ros::Rate(1) );
  int imufreq=1,previmufreq=1;

  MPU6050 accelgyro;
  accelgyro.initialize();


  double AccGyro[7];

  // int count = 0;
  while (ros::ok())
  {

    // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelgyro.getScaledaccgyro_timestamped(AccGyro);
    // std::cout << ax << " " << ay << " " << az << " " << gx << " " << gy << " " << gz << std::endl;

    rosutils::RawImu imu;
    imu.acc[0] = AccGyro[1];
    imu.acc[1] = AccGyro[2];
    imu.acc[2] = AccGyro[3];

    imu.gyro[0] = AccGyro[4];
    imu.gyro[1] = AccGyro[5];
    imu.gyro[2] = AccGyro[6];

    imu_pub.publish(imu);

    ROS_INFO("Sending IMU %f %f %f %f %f %f",AccGyro[1],AccGyro[2],AccGyro[3],AccGyro[4],AccGyro[5],AccGyro[6]);

    ros::spinOnce();

    loop_rate_ptr->sleep();

    nh.getParam("imufreq", imufreq);
    
    if (imufreq!=previmufreq){
      loop_rate_ptr.reset(new ros::Rate(imufreq));
      previmufreq = imufreq;
    }

    // ++count;
  }


  return 0;
}
