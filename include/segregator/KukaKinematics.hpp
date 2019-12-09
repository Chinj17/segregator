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
 * @file KukaKinematics.hpp
 * @brief This is the header file of the Kuka Kinematics class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */

#ifndef INCLUDE_SEGREGATOR_KUKAKINEMATICS_HPP_
#define INCLUDE_SEGREGATOR_KUKAKINEMATICS_HPP_

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <vector>
#include <string>

/*
 * @brief KukaKinematics is a class used for working with the Kuka robot
 *        manipulation
 */
class KukaKinematics {
 private:
    // ROS node handle
     ros::NodeHandle n;
    // Publisher to robot joints for moving the robot
     ros::Publisher jointPublisher;
    // variable to send final motion commads to the robot
    trajectory_msgs::JointTrajectory jointCommands;
    // number of joints in Kuka robot
    const unsigned int numJoints = 7;
    // variable to store joint values
    double posJoints[11][7];

    /*
     * @brief This is a private method of this class. Used to initialize all the
     *        attributes of trajectory message.
     *
     * @param This method does not take any inputs.
     *
     * @return This method does not return any argument.
     */
    void initializeTrajectoryPoint();

 public:
  // Define the various states of the robot
  enum States {HOME, LEFT_SLAB, RIGHT_SLAB, LEFT_CASE_POS_1,
                LEFT_CASE_POS_2, RIGHT_CASE_POS_1, RIGHT_CASE_POS_2,
                HOME_LEFT_SLAB, HOME_RIGHT_SLAB, HOME_BACK_SLAB,
                BACK_CASE_POS_1};
  // Define possible robot states as string
  std::vector<std::string> statesStr =  {"Home", "Left slab", "Right slab",
                                          "Left case 1", "Left case 2",
                                          "Right case 1", "Right case 2",
                                          "Home left", "Home Right",
                                          "Home Back", "Back case 1"};

    /*
     * @brief This is the constructor for the class.
     *
     * @param This is a constructor so it does not take any inputs.
     *
     * @return This is a constructor so it does not return anything.
     */
     KukaKinematics();

     /*
      * @brief This is the first method of the class. It is used to move the robot
      *        to the desired position
      *
      * @param Input to this method is the position that needs to be reached.
      *
      * @result This method does not return anything.
      */
     void sendRobotToPos(const States &);

     /*
      * @brief This is the destructor for the class.
      *
      * @param This is a destructor so it does not take any inputs.
      *
      * @return This is a destructor so it does not return anything.
      */
     ~KukaKinematics();
};

#endif  // INCLUDE_SEGREGATOR_KUKAKINEMATICS_HPP_
