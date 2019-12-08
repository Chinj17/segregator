#include <gtest/gtest.h>
#include "KukaKinematics.hpp"
#include "Gripper.hpp"

TEST(KukaGripperTest, testGetGripperState) {
    //
    KukaGripper test;

    //
    ASSERT_FALSE(test.getGripperState());
}

TEST(KukaGripperTest, testGripperToggle) {
    //
    KukaGripper test;
    KukaKinematics robot;

    //
    robot.sendRobotToPos(robot.HOME);
    robot.sendRobotToPos(robot.RIGHT_SLAB);

    //
    test.gripperToggle(true);

    //
    ASSERT_TRUE(test.getGripperState());

    //
    test.gripperToggle(false);

    //
    robot.sendRobotToPos(robot.HOME);
}
