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
 * @file Gripper.hpp
 * @brief This is the header file of the Kuka Gripper class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */

#ifndef INCLUDE_SEGREGATOR_GRIPPER_HPP_
#define INCLUDE_SEGREGATOR_GRIPPER_HPP_

#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <iostream>

/*
 * @brief This is the Kuka Gripper class for controlling the state of
 *        the vacuum gripper
 */

class KukaGripper {
 private:
    // ROS node handle
     ros::NodeHandle n;
    // Service clients for switching the gripper ON and OFF
     ros::ServiceClient gripperOn, gripperOff;
    // Subscriber to check the gripper state
     ros::Subscriber gripperSubscriber;
    // Variable to check the gripper state
     bool gripperState = false;

    /*
     * @brief This is a private method of this class. It checks the state of
     *        the gripper
     *
     * @param Input is the message being published by the grasping topic
     *
     * @return This method returns nothing
     */
     void gripperCallback(const std_msgs::Bool &);

  public:
    /*
     * @brief This is the constructor for the class.
     *
     * @param No inputs as it is a constructor. It creates a subscriber
     *        and two service clients.
     *
     * @return This is a constructor so it returns nothing.
     */
     KukaGripper();

    /*
     * @brief This is the first method of the class. It is used to change the
              state of the vacuum gripper
     *
     * @param This method takes state of the gripper.
     *
     * @return This method does not return anything.
     */
     void gripperToggle(const bool &);

    /*
     * @brief This is the second method of the class. It is used to get the
     *        gripper state
     *
     * @param This method does not take any input.
     *
     * @return This method returns nothing.
     */
     bool getGripperState();

    /*
     * @brief This is the destructor for the class.
     *
     * @param This is a destructor so it does not take any inputs.
     *
     * @return This is a destructor so it returns nothing.
     */
     ~KukaGripper();

};

#endif  // INCLUDE_SEGREGATOR_GRIPPER_HPP_
