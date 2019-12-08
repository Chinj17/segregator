#include "Gripper.hpp"

// constructor
KukaGripper::KukaGripper() {
    // subscriber
     gripperSubscriber = n.subscribe("/robot/left_vacuum_gripper/grasping", 10, &KukaGripper::gripperCallback, this);
    // On State
     gripperOn = n.serviceClient<std_srvs::Empty>("/robot/left_vacuum_gripper/on");
    // off State
     gripperOff = n.serviceClient<std_srvs::Empty>("/robot/left_vacuum_gripper/off");
}

void KukaGripper::gripperToggle(const bool & state) {
    // Call the gripper service
    std_srvs::Empty empty;
    if (state) {
        gripperOn.call(empty);
        while (!gripperState) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::Duration(2).sleep()
    } else {
        gripperOff.call(empty);
        while (gripperState) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::Duration(2).sleep();
    }
  }
}

bool KukaGripper::getGripperState() {
    return gripperState;
}

void KukaGripper::gripperCallback(const std_msgs::Bool & state) {
    gripperState = state.data;
}
