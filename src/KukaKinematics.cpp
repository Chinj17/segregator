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
 * @brief It is the chain making function which contains the forward kinematics solver
 */
void KukaKinematics::makeChain() {
  //Define LWR chain
  KDL::Chain chain;
  //base
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::None),
                   KDL::Frame::DH_Craig1989(0, 0, 0.33989, 0)));
  //joint 1
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
  //joint 2
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0.40011, 0)));
  //joint 3
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
  //joint 4
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0.40003, 0)));
  //joint 5
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, -M_PI_2, 0, 0)));
  //joint 6
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, M_PI_2, 0, 0)));
  //joint 7 (with flange adapter)
  chain.addSegment(
      KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                   KDL::Frame::DH_Craig1989(0, 0, 0.12597, 0)));
  kinematicChain = chain;
  fkSolver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  ikSolverVel.reset(new KDL::ChainIkSolverVel_pinv(chain));
  KDL::ChainFkSolverPos_recursive fkSolver = KDL::ChainFkSolverPos_recursive(
      chain);
  KDL::ChainIkSolverVel_pinv ikSolverVel = KDL::ChainIkSolverVel_pinv(chain);
  ikSolver.reset(
      new KDL::ChainIkSolverPos_NR(chain, fkSolver, ikSolverVel, 100, 1e-4));
}
/**
 * @brief This function is to get the joint numners of the manipulator
 */
void KukaKinematics::getJointNums() {
  numJoints = kinematicChain.getNrOfJoints();
  jointPosKdl = KDL::JntArray(numJoints);
  newJointPosKdl = KDL::JntArray(numJoints);
}
/**
 * @brief This function is to initialize joints as per the degree of freedom
 */
void KukaKinematics::initializeTrajectoryPoint() {
  jointCommands.joint_names.push_back("iiwa_joint_1");
  jointCommands.joint_names.push_back("iiwa_joint_2");
  jointCommands.joint_names.push_back("iiwa_joint_3");
  jointCommands.joint_names.push_back("iiwa_joint_4");
  jointCommands.joint_names.push_back("iiwa_joint_5");
  jointCommands.joint_names.push_back("iiwa_joint_6");
  jointCommands.joint_names.push_back("iiwa_joint_7");
  jointCommands.header.seq = 0;
  jointCommands.header.stamp = ros::Time::now();
  jointCommands.header.frame_id = "";
}
/**
 * @brief This function is for initializing home positition joint configuration
 */
void KukaKinematics::initializeHomePos() {
  for (int i = 0; i < numJoints; ++i) {
    if (i == 0)
      homePos.positions.push_back(0);  //1.3);
    if (i == 1)
      homePos.positions.push_back(1.0);  //0.0);
    if (i == 2)
      homePos.positions.push_back(1.0);  //0.0);
    if (i == 3)
      homePos.positions.push_back(-1.57);  //-1.57);
    if (i == 4)
      homePos.positions.push_back(0.0);  //0.0);
    if (i == 5)
      homePos.positions.push_back(1.0);  //1.57);
    if (i == 6)
      homePos.positions.push_back(0);
  }
  homePos.time_from_start = ros::Duration(1.0);
}

/**
 * @brief It is a subscriber to the Kuka joint values.
 * @params pass the joint state
 */

void KukaKinematics::getJoints(sensor_msgs::JointState & jointState) {
  // ROS_INFO_STREAM("yo"<<jointStates->position[1]);
  jointStates = *msg;
}

void KukaKinematics::initializeJointsKDL() {
  for (int i = 0; i < numJoints; ++i) {
    jointPosKdl(i) = 0.2;
    newJointPosKdl(i) = 0.2;
  }
}

void KukaKinematics::initializeJointsSub() {
  for (int i = 0; i < numJoints; ++i) {
    jointStates.position.push_back(0.2);
  }
}
trajectory_msgs::JointTrajectory KukaKinematics::homeRobot() {
  trajectory_msgs::JointTrajectory jointCmd;
  jointCmd = jointCommands;
  jointCmd.points[0] = homePos;
  jointCmd.header.seq = 0;
  jointCmd.header.stamp = ros::Time::now();
  jointCmd.header.frame_id = "";
  return jointCmd;
}

/**
 * @brief It solves the inverse kinematic problem for the kuka robot using Moveit with its OMPS planner
 */

KDL::Frame KukaKinematics::evalKinematicsFK() {
  KDL::Frame cartPos;
  for (int k = 0; k < numJoints; ++k) {
    jointPosKdl(k) = jointStates.position[k];
  }
  // ROS_INFO_STREAM("yo"<<jointStates.position[1]);
  kinematicsStatus = fkSolver->JntToCart(jointPosKdl, cartPos);
  currCartPos = cartPos;
  return cartPos;
}
/**
 * @brief This is the second method of the class. It solves the inverse kinematic
 * @brief problem for the kuka robot using KDL.
 * @param cartpos
 * @return
 */
KDL::JntArray KukaKinematics::evalKinematicsIK(KDL::Frame cartpos) {
  ROS_INFO_STREAM("1");
  ROS_INFO_STREAM("cartpos" << cartpos.p[1]);
  ROS_INFO_STREAM("currjoint" << jointPosKdl(1));
  ROS_INFO_STREAM("currjoint" << newJointPosKdl(1));
  int ret = ikSolver->CartToJnt(jointPosKdl, cartpos, newJointPosKdl);
  ROS_INFO_STREAM("2");
  return newJointPosKdl;
}

/**
 * @brief It normalizes the output from the inverse kinematic solver.
 * @params pass array of joint pose
 */

trajectory_msgs::JointTrajectoryPoint KukaKinematics::normalizePoints(
    KDL::JntArray) {
  trajectory_msgs::JointTrajectoryPoint point_;
  // joints can move between -+: 172,120,172,120,172,120,170
  //double joint_bounds[] = {3.002, 2.0944,3.002, 2.0944,3.002, 2.0944, 3.002};
  for (int i = 0; i < numJoints; ++i) {
    while (newJointPosKdl(i) > M_PI)
      newJointPosKdl(i) -= 2 * M_PI;
    while (newJointPosKdl(i) < -M_PI)
      newJointPosKdl(i) += 2 * M_PI;
    point_.positions[i] = newJointPosKdl(i);
  }
  return point_;
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

