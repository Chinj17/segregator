**
 * @file KukaKinematics.hpp
 * Copyright [2019] [Kamakshi Jain] - driver
 * @date Nov 29, 2019
 * @brief This is the declaration of the KukaKinematics class
 */

#ifndef INCLUDE_KUKAKINEMATICS_HPP_
#define INCLUDE_KUKAKINEMATICS_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

/*
 * @brief KukaKinematics is a class used for working with the Kuka robot
 *        manipulation
 */
class KukaKinematics {
 private:
    ros::NodeHandle n;
    sensor_msgs::JointState jointInfo;
    KDL::Chain kinematicChain;
    KDL::ChainFkSolverPos_recursive fkSolver;
    KDL::ChainIkSolverVel_pinv ikSolver;
    unsigned int numJoints;
    KDL::JntArray jointPos;
    KDL::Frame currCartesianPos;
    trajectory_msgs::JointTrajectory jointCommands;

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
    void getJoints(sensor_msgs::JointState &);

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
    KDL::JntArray evalKinematics(KDL::Frame, KDL::JntArray);

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
    trajectory_msgs::JointTrajectory evalKinematics(std::vector<double>);

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

    /*
     * @brief This is the destructor for the class
     */
    ~KukaKinematics();
};

#endif  // INCLUDE_KUKAKINEMATICS_HPP_
