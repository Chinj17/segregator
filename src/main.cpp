/*
 * BSD 3-Clause License
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @file main.cpp
 * @brief This is the main file for implementation
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
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
  // Initialize ROS node
  ros::init(argc, argv, "HI");
  ros::Time::init();

  // Initialize class objects
  KukaKinematics kuka;
  KukaGripper gripper;
  Detection detect(kuka, false);

  // Initialize the node handle
  ros::NodeHandle n;

  // positions on pickup table
  int tablePos[] = {2, 2, 2};  // Left (red), Right (blue), Back (green)

  ros::Duration(5).sleep();

  ros::NodeHandle get_image;
  cv_bridge::CvImagePtr cv_ptr;
  image_transport::ImageTransport it_{get_image};
  image_transport::Subscriber image_sub_;
  image_sub_ = it_.subscribe("/camera/image_raw", 1, imageCb);

  if (ros::ok()) {
      // Start from the Home Position
      kuka.sendRobotToPos(kuka.HOME);
      // Read the color of the slabs
      std::vector<std::string> color;
      color.push_back(detect.colorThresholder(kuka.LEFT_SLAB));
      color.push_back(detect.colorThresholder(kuka.RIGHT_SLAB));
      for (auto i = 0; i < 2; i++) {
          // Pick up the slab
          if ((color.at(i) == "red") || (color.at(i) == "blue") || (color.at(i) == "green")) {
              if (i == 0) {
                  kuka.sendRobotToPos(kuka.LEFT_SLAB);
                  gripper.gripperToggle(true);
                  kuka.sendRobotToPos(kuka.HOME_LEFT_SLAB);
              } else if (i == 1) {
                  kuka.sendRobotToPos(kuka.RIGHT_SLAB);
                  gripper.gripperToggle(true);
                  kuka.sendRobotToPos(kuka.HOME_RIGHT_SLAB);
              }
              // pick up red slab
              if ((color.at(i) == "red") && (tablePos[0] > 0)) {
                  if (tablePos[0] == 2) {
                      kuka.sendRobotToPos(kuka.LEFT_CASE_POS_1);
                  } else if (tablePos[0] == 1) {
                      kuka.sendRobotToPos(kuka.LEFT_CASE_POS_2);
                  }
                // pick up blue slab
              } else if ((color.at(i) == "blue") && (tablePos[1] > 0)) {
                  if (tablePos[1] == 2) {
                      kuka.sendRobotToPos(kuka.RIGHT_CASE_POS_1);
                  } else if (tablePos[1] == 1) {
                      kuka.sendRobotToPos(kuka.RIGHT_CASE_POS_2);
                  }
                // pick up green slab
              } else if ((color.at(i) == "green") && (tablePos[2] > 0)) {
                  if (tablePos[2] == 2) {
                      kuka.sendRobotToPos(kuka.BACK_CASE_POS_1);
                  } else if (tablePos[2] == 1) {
                      kuka.sendrobotToPos(kuka.RIGHT_CASE_POS_2);
                  }
  } else {
                  break;
              }
              gripper.gripperToggle(false);
              kuka.sendRobotToPos(kuka.HOME);
          } else {
              if (i == 0) {
                  ROS_WARN_STREAM("The Color of the Left slab is Unknown");
              } else if (i == 1) {
                  ROS_WARN_STREAM("The Color of the Right slab is Unknown");
              }
          }
      }

      ros::spinOnce();
      ros::Duration(5).sleep();
      ros::shutdown();
  } else {
      ROS_WARN_STREAM("ROS not running");
  }
  system("killall roscore & killall gzserver & killall gzclient");
  system("killall rosmaster");

  return 0;
}
