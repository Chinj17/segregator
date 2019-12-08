/**
 * @file main.cpp
 * Copyright [2019] [Kamakshi Jain] - driver
 * @date Nov 29, 2019
 * @brief This is the implementation of the segregation algorithm
 */

#include <iostream>
#include "KukaKinematics.hpp"
#include "Gripper.hpp"
#include "Detection.hpp"
#include "sensor_msgs/Image.h"

static const std::string OPENCV_WINDOW = "Image window";
int flag = 0;

void imageCb(const sensor_msgs::ImageConstPtr& msg) {

       cv_bridge::CvImagePtr cv_ptr;
       try
       {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
       }
       cv::imshow(OPENCV_WINDOW, cv_ptr->image);
       cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "HI");
  ros::Time::init();

  // Initialize class objects
  KukaKinematics kuka;
  KukaGripper gripper;
  Detection detect(kuka, false);
// ros::Duration(5).sleep();

while (ros::ok()) {
ros::Duration(0.0011).sleep();

ROS_INFO_STREAM("hi");
cartpos = ku.evalKinematicsFK();
inv = ku.evalKinematicsIK(cartpos);
ROS_INFO_STREAM("hi");
ROS_INFO_STREAM("Hi"<< inv(1));
ros::spinOnce();
}

  return 0;
}
