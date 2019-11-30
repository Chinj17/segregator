*
 * @file testDetection.cpp
 * 
 * @brief It defines rostest and gtest for Detection class.
 *
 * Copyright [2019] Kamakshi Jain
 */

#include <gtest/gtest.h>
#include "KukaKinematics.hpp"
#include "Detection.hpp"

/**
 * @brief This is the google test for the first method of the class.
 */ 
TEST(DetectionTest, testColorThresholder) {
    // Initialize the Detection and KukaKinematics object
    KukaKinematics robot;
    Detection test(robot, false);

    // Initialize node handle
    ros::NodeHandle n;

    // Initialize image subscriber
    image_transport::ImageTransport imgT(n);
    auto imageSubscriber_ = imgT.subscribe("/camera/image_raw", 1,
                                                &Detection::readImg, &test);
    ros::Duration(1).sleep();
    ros::spinOnce();
    ros::Duration(1).sleep();

    // Check if the left disc is red colored
    auto color = test.colorThresholder(robot.LEFT_DISK);
    EXPECT_FALSE(color.compare("red"));

    // Check if the right disc is blue colored
    color = test.colorThresholder(robot.RIGHT_DISK);
    EXPECT_FALSE(color.compare("blue"));

    // Check if the any other disc is read
    color = test.colorThresholder(robot.HOME);
    EXPECT_TRUE(color.empty());
}
