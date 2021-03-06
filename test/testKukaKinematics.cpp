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
 * @file testKukaKinematics.cpp
 * @brief This is the implementation file to test the Kuka KukaKinematics class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */
#include <gtest/gtest.h>
#include "KukaKinematics.hpp"

trajectory_msgs::JointTrajectory command;

// Create a callback function
void callback(const trajectory_msgs::JointTrajectory & pubCommand) {
    command = pubCommand;
}

/*
 * @brief This is the google test for the first method of the class.
 */
TEST(KukaKinematicsTest, testSendRobotToPos) {
    // Initialize a KukaKinematics class object
    KukaKinematics test;

    // Initialize the node handle
    ros::NodeHandle n;

    // Initialize the home position coordinates
    double posJoints[] = {0, 0, 0, 0, 0, 0, 0};

    // Create a subscriber
    auto gripperSubscriber = n.subscribe(
        "/iiwa/PositionJointInterface_trajectory_controller/command", .10,
                                                                    callback);

    // Move the robot to a particular position
    test.sendRobotToPos(test.HOME);
    ros::spinOnce();

    // Test if the command being sent is correct
    ASSERT_EQ(0, command.header.seq);
    ASSERT_EQ(0, command.header.frame_id.compare("Home"));
    auto i = 0;
    while (i < 1) {
      EXPECT_EQ(0, 0); i++;
    }
}
