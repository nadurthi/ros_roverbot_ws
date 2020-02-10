#include "ros/ros.h"
#include <iostream>
#include <ros/console.h>
#include <math.h>       /* fabs */

// #include "std_msgs/String.h"
// #include "sensor_msgs/Imu.h"
#include "rosutils/RawImu.h"
#include "roverbot/MPU6050.h"
#include <memory>

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

std::unique_ptr<ros::Rate> loop_rate_ptr;

void dynamic_cfg_callback(roverbot::RoverbotConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d %s", 
            config.sonar_freq, 
            config.imu_freq, 
            config.cam_freq, 
            config.cam_on_off?"True":"False"
            );

      if (config.imu_freq<0){
        loop_rate_ptr.reset(new ros::Rate(1/std::fabs(config.imu_freq)));
      }
      else{
        loop_rate_ptr.reset(new ros::Rate(config.imu_freq));
      }
      
      

}


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "roverbot_imu");

  ros::NodeHandle nh("roverbot");


  ros::Publisher imu_pub = nh.advertise<rosutils::RawImu>("rover_imu", 5);

  dynamic_reconfigure::Server<roverbot::RoverbotConfig> server;
  dynamic_reconfigure::Server<roverbot::RoverbotConfig>::CallbackType f;
  f = boost::bind(&dynamic_cfg_callback, _1, _2);
  server.setCallback(f);


  loop_rate_ptr.reset( new ros::Rate(1) );

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
    imu.linear_acceleration.x = AccGyro[1];
    imu.linear_acceleration.y = AccGyro[2];
    imu.linear_acceleration.z = AccGyro[3];

    imu.angular_velocity.x = AccGyro[4];
    imu.angular_velocity.y = AccGyro[5];
    imu.angular_velocity.z = AccGyro[6];

    imu_pub.publish(imu);

    // ROS_DEBUG("Sending IMU %f %f %f %f %f %f",AccGyro[1],AccGyro[2],AccGyro[3],AccGyro[4],AccGyro[5],AccGyro[6]);

    ros::spinOnce();

    loop_rate_ptr->sleep();

  }


  return 0;
}
