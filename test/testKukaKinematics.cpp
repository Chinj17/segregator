/**
 * @file KukaKinematics.cpp
 * Copyright [2019] [Kamakshi Jain] - driver
 * @date Nov 29, 2019
 * @brief This defines rostestand gtest of the KukaKinematics class
 */

#include <gtest/gtest.h>
#include "KukaKinematics.hpp"

trajectory_msgs::JointTrajectory command;

// Create a callback function
void callback(const trajectory_msgs::JointTrajectory & pubCommand) {
    command = pubCommand;
}

// This is the google test for the first method of the class.
TEST(KukaKinematicsTest, testSendRobotToPos) {
    // Initialize a KukaKinematics class object
    KukaKinematics test;

    // Initialize the node handle
    ros::NodeHandle n;

    // Initialize the home position coordinates
    double posJoints[] = {2.918232931819958, -0.03327128068635865,
                          -1.168222646872703, -1.2265077682929597,
                          -0.028973511280637965, 1.9434842643815777,
                          -1.402031709125911};

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
    for (auto i = 0; i < 7; i++)
        ASSERT_EQ(posJoints[i], command.points[0].positions[i]);
}
