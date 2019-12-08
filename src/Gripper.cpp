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
