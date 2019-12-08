/*
 * BSD 3-Clause License
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * @file Gripper.cpp
 * @brief This is the implementation file of the Kuka Gripper class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */

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
        ros::Duration(2).sleep();
    } else {
        gripperOff.call(empty);
        while (gripperState) {
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
        ros::Duration(2).sleep();
    }
}

bool KukaGripper::getGripperState() {
    return gripperState;
}

void KukaGripper::gripperCallback(const std_msgs::Bool & state) {
    gripperState = state.data;
}

KukaGripper::~KukaGripper() {
    // Can put some warning
}
