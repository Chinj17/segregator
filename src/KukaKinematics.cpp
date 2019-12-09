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
 * @file KukaKinematics.cpp
 * @brief This is the implementation file of the Kuka Kinematics class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */

#include "KukaKinematics.hpp"

/**
 * @brief This is the constructor for the class
 */
KukaKinematics::KukaKinematics() : posJoints{{0, 0, 0, 0, 0, 0, 0},
    {-0.49584801156347513, -0.7282973144476284, -1.0608509979595278,
    0.8929195071031097, -2.4641855236659262, 1.7728411547958132,
    -1.1338010045240487}, {0.6792482988825581,
    0.8599605609631125, -2.5690240012934122, 0.43208146096807276,
    -0.4863815052273104, -1.8754601535685547, -2.231439350038941},
    {0.21351211197587183, -0.7330547797325959, 1.0832547519372078,
    0.8793723299536111, -0.6545890640361121, -1.813080768567132,
    0.9917921256510782}, {0.8988227072345678, -0.5531091106745061,
    -1.3644584598885636, 1.2129732571057295, -2.5863039320840686,
    1.6961754475543316, -1.8061755708947427}, {2.7575663568670254,
    -0.5858281224554007, -0.440354297731969, 0.941934191128194,
    -2.8727544970384757, 1.6237417994837084, -0.7114584416147531},
    {2.554736326027861, -0.41894943120107, 0.9373442781635903,
    1.217784288805702, -0.3350002830663925, -1.6425172754590998,
    0.38282955332744617}, {0.11351211197587183, -0.3530547797325959, 0.5,
    0.4353723299536111, -0.3255890640361121, -1.813080768567132,
    0.4517921256510782}, {0.44988227072345678, 0.2531091106745061,
    0.6644584598885636, -0.6129732571057295, -1.2563039320840686,
    0.8961754475543316, 0.9061755708947427}, {0, 0, 0, 0, 0, 0, 0},
    {-1.570796327, 0.2531091106745061, 0.6644584598885636,
    -0.6129732571057295, -1.2563039320840686, 0.8961754475543316,
     0.9061755708947427}} {
    // Initialize the trajectory message attributes
    initializeTrajectoryPoint();
    // Initialize the publisher
    jointPublisher = n.advertise<trajectory_msgs::JointTrajectory>(
             "/iiwa/PositionJointInterface_trajectory_controller/command", .10);
}

/*
 * @brief This is a private method of this class. Used to initialize all the
 *        attributes of trajectory message.
 *
 * @param This method does not take any inputs.
 *
 * @return This method does not return any argument.
 */
void KukaKinematics::initializeTrajectoryPoint() {
  // Initailize the attributes of the trajectory message
    auto i = 1;
    while (i <= numJoints) {
        jointCommands.joint_names.push_back("iiwa_joint_" + std::to_string(i));
        i++;
    }
    jointCommands.header.seq = 0;
    jointCommands.header.stamp = ros::Time::now();
    jointCommands.header.frame_id = "";
    jointCommands.points.resize(1);
    jointCommands.points[0].positions.resize(numJoints);
    jointCommands.points[0].time_from_start = ros::Duration(3.0);
}

/*
 * @brief This is the first method of the class. It is used to move the robot
 *        to the desired position
 *
 * @param Input to this method is the position that needs to be reached.
 *
 * @result This method does not return anything.
 */
void KukaKinematics::sendRobotToPos(const States & state) {
    // Define the position
    auto num = static_cast<int>(state);

    // Define the jointCommands variable
    jointCommands.header.stamp = ros::Time::now();
    jointCommands.header.frame_id = statesStr.at(num);
    auto i = 0;
    while (i <= numJoints) {
        jointCommands.points[0].positions[i] = posJoints[num][i];
        i++;
    }
    // Publish the joint Commands
    jointPublisher.publish(jointCommands);
    ros::spinOnce();
    ros::Duration(3).sleep();  // Give the robot time to reach desired position
}

/*
 * @brief This is the destructor for the class.
 *
 * @param This is a destructor so it does not take any inputs.
 *
 * @return This is a destructor so it does not return anything.
 */
KukaKinematics::~KukaKinematics() {
    ROS_WARN_STREAM("Robot Motion Control Module has been Shut Down");
}
