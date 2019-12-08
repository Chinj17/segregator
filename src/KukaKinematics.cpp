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
KukaKinematics::KukaKinematics() : posJoints{{0,0,0,0,0,0,0},
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
    0.38282955332744617},{0.11351211197587183, -0.3530547797325959, 0.5,
    0.4353723299536111, -0.3255890640361121, -1.813080768567132,
    0.4517921256510782},{0.44988227072345678, 0.2531091106745061,
    0.6644584598885636, -0.6129732571057295, -1.2563039320840686,
    0.8961754475543316, 0.9061755708947427}, {0, 0, 0, 0, 0, 0, 0}, {-1.570796327, 0.2531091106745061, 0.6644584598885636, -0.6129732571057295, -1.2563039320840686,
    0.8961754475543316, 0.9061755708947427}} {
    // Initialize the trajectory message attributes
    initializeTrajectoryPoint();
    // Initialize the publisher
    jointPublisher = n.advertise<trajectory_msgs::JointTrajectory>("/iiwa/PositionJointInterface_trajectory_controller/command", .10);
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

void KukaKinematics::getJoints(const sensor_msgs::JointState::ConstPtr& msg) {
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
