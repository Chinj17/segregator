/**
 * @file KukaKinematics.cpp
 * Copyright [2019] Kamakshi Jain, sayan brahma
 * @date Nov 29, 2019
 * @brief This is the implementation of the KukaKinematics class
 */

#include "KukaKinematics.hpp"

/**
 * @brief This is the constructor for the class
 */
KukaKinematics::KukaKinematics() {
  makeChain();
  getJointNums();
  initializeTrajectoryPoint();
  initializeHomePos();
  jointPosKdl = KDL::JntArray(KukaKinematics::numJoints);
  newJointPosKdl = KDL::JntArray(KukaKinematics::numJoints);
  initializeJointsKDL();
  initializeJointsSub();
}
/**
 * @brief It is a subscriber to the Kuka joint values.
 * @params pass the joint state
 */

void KukaKinematics::getJoints(sensor_msgs::JointState & jointState) {

}

/**
 * @brief It solves the inverse kinematic problem for the kuka robot using KDL
 * @params pass frame
 * @params pass array of joint pose
 */

KDL::JntArray KukaKinematics::evalKinematics(KDL::Frame frame,
                                             KDL::JntArray jointArr) {

}

/**
 * @brief It solves the forward kinematic using Moveit with OMPL planner
 * @params pass the joint state
 */
trajectory_msgs::JointTrajectory KukaKinematics::evalKinematics(
    std::vector<double> jointState) {

}
/**
 * @brief It normalizes the output from the inverse kinematic solver.
 * @params pass array of joint pose
 */

trajectory_msgs::JointTrajectoryPoint KukaKinematics::normalizePoints(
    KDL::JntArray jointArr) {

}

/**
 * @brief It checks whether the inverse kinematic solver ran successfully or not.
 */
bool KukaKinematics::checkKinematicStatus() {

}

/**
 * @brief This is the destructor for the class
 */
KukaKinematics::~KukaKinematics() {

}

