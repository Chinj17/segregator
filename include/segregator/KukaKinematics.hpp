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
    /*
     * @brief This is the constructor for the class
     */
    KukaKinematics();

    /*
     * @brief This is the first method of the class. It is a subscriber to
     *        the Kuka joint values.
     *
     * @param This function takes the joint state as input by reference.
     *
     * @result This function does not return anything. The new joint values are
     *         stored in the variable taken as input.
     */
    void getJoints(const sensor_msgs::JointState::ConstPtr&);

    /*
     * @brief This is the second method of the class. It solves the inverse
     *        kinematic problem for the kuka robot using KDL.
     * Note that, this code is a part of another project which uses a haptic
     * device. As MoveIt does not have support for this haptic device, KDL has
     * been implemented instead.
     *
     * @param The first parameter to this function is the desired frame
     *        coordinates - rotation and translation.
     * @param The second parameter to this function is the current joint
     *        coordinates. This is required to get the correct path from the
     *        start position to the desired position.
     *
     * @result This function returns the new joint coordinates required to
     *         achieve the desired position.
     */
    KDL::Frame evalKinematicsFK();

    /*
     * @brief This is the third method of the class. It solves the forward
     *        kinematic problem for the kuka robot using MoveIt with its
     *        OMPL planner.
     *
     * @param This function takes the desired joint state as input.
     *
     * @result This function returns joint trajectory that the robot need to
     *         follow to reach the desired joint state.
     */
    KDL::JntArray evalKinematicsIK(KDL::Frame);

    /*
     * @brief This is the fourth method of the class. It normalizes the output
     *        from the inverse kinematic solver. This is required as the joint
     *        angles given by the KDL solver may be greater than 2*pi or less
     *        than -2*pi.
     *
     * @param This function takes the computed joint state as input.
     *
     * @result This function returns the normalized joint trajectory.
     */
    trajectory_msgs::JointTrajectoryPoint normalizePoints(KDL::JntArray);

    /*
     * @brief This is the fifth method of the class. It checks whether the
     *        inverse kinematic solver ran successfully or not.
     *
     * @param This function does not take any input.
     *
     * @result This function returns true if the inverse kinematic solver ran
     *         successfully, otherwise it returns false.
     */
    bool checkKinematicStatus();

    trajectory_msgs::JointTrajectory homeRobot();

    /*
     * @brief This is the destructor for the class
     */
    ~KukaKinematics();
};

#endif  // INCLUDE_KUKAKINEMATICS_HPP_
