/**
 * @file KukaKinematics.hpp
 * Copyright [2019] [Kamakshi Jain] - driver
 * @date Nov 29, 2019
 * @brief This is the declaration of the KukaKinematics class
 */

#ifndef INCLUDE_KUKAKINEMATICS_HPP_
#define INCLUDE_KUKAKINEMATICS_HPP_

#include <ros/ros.h>
#include <iostream>
// #include <vector>
// #include <string>
// #include <opencv2/opencv.hpp>
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
/*
 * @brief KukaKinematics is a class used for working with the Kuka robot
 *        manipulation
 */
class KukaKinematics {
 private:
    //
     ros::NodeHandle n;
    //
     ros::Publisher jointPublisher;
    // final motion commads sent to the robot
    trajectory_msgs::JointTrajectory jointCommands;
    //
    const unsigned int numJoints = 7;
    //
    double posJoints[11][7];

    /**
     * @brief <brief>
     * @param [in] <name> <parameter_description>
     * @return <return_description>
     * @details <details>
     */
    void initializeTrajectoryPoint();

 public:

   // Define the various states of the robot
   enum States {HOME, LEFT_SLAB, RIGHT_SLAB, LEFT_CASE_POS_1,
                LEFT_CASE_POS_2, RIGHT_CASE_POS_1, RIGHT_CASE_POS_2, HOME_LEFT_SLAB, HOME_RIGHT_SLAB, HOME_BACK_SLAB, BACK_CASE_POS_1};
   // Define possible robot states as string
   std::vector<std::string> statesStr =  {"Home", "Left slab", "Right slab",
                                          "Left case 1", "Left case 2",
                                          "Right case 1", "Right case 2","Home left", "Home Right", "Home Back", "Back case 1"};
    /*
     * @brief This is the constructor for the class
     */
     KukaKinematics();

    /*
     * @brief This is the first method of the class.
     *
     * @param
     *
     * @return
     */
     void sendRobotToPos(const States &);

    /*
     * @brief This is the destructor for the class
     */
     ~KukaKinematics();
};

#endif  // INCLUDE_KUKAKINEMATICS_HPP_
