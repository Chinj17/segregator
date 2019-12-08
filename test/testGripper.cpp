#include <gtest/gtest.h>
#include "KukaKinematics.hpp"
#include "Gripper.hpp"

TEST(KukaGripperTest, testGetGripperState) {
    //
    KukaGripper test;

    //
    ASSERT_FALSE(test.getGripperState());
}
