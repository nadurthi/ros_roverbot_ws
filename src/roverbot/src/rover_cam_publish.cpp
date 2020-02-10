#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <memory>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <roverbot/RoverbotConfig.h>
#include <math.h>       /* fabs */



bool cam_switch = 0;
float cam_freq =0;
int video_source = 0;


std::unique_ptr<ros::Rate> loop_rate_ptr;


void dynamic_cfg_callback(roverbot::RoverbotConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %d %d %s", 
            config.sonar_freq, 
            config.imu_freq, 
            config.cam_freq, 
            config.cam_on_off?"True":"False"
             );
      // cout << 
      cam_freq = config.cam_freq;
      if (cam_freq<0){
        cam_freq=1/std::fabs(cam_freq);
      }

      cam_switch=config.cam_on_off;

      loop_rate_ptr.reset( new ros::Rate(cam_freq) );
}




int main(int argc, char** argv)
{

  
  

  ros::init(argc, argv, "roverbot_cam0");
  ros::NodeHandle nh("roverbot");



  dynamic_reconfigure::Server<roverbot::RoverbotConfig> server;
  dynamic_reconfigure::Server<roverbot::RoverbotConfig>::CallbackType f;
  f = boost::bind(&dynamic_cfg_callback, _1, _2);
  server.setCallback(f);




  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 30);



  // cv::VideoCapture cap(video_source);
  // if(!cap.isOpened()) 
  //   return 1;
  
  // cap.release ()

  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  loop_rate_ptr.reset( new ros::Rate(1) );

  cv::VideoCapture cap;
  

  while (ros::ok()) {
    if(cam_switch==true && !cap.isOpened()){
        cap.open(video_source);
        ROS_INFO("Started cam");
      }

      if(cam_switch==false && cap.isOpened()){
        cap.release();
        ROS_INFO("ended cam");
      }

    if (cap.isOpened()==true){
      cap >> frame;
      // Check if grabbed frame is actually full with some content
      if(!frame.empty()) {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        cv::waitKey(1);
        ROS_INFO("framed cam");
      }
    }
    ros::spinOnce();

    loop_rate_ptr->sleep();


 

  }
}