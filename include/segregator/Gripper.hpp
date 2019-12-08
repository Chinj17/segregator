#ifndef INCLUDE_SEGREGATOR_GRIPPER_HPP_
#define INCLUDE_SEGREGATOR_GRIPPER_HPP_

#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <iostream>

#endif  // INCLUDE_SEGREGATOR_GRIPPER_HPP_
class KukaGripper {
 private:
    //
     ros::NodeHandle n;
    //
     ros::ServiceClient gripperOn, gripperOff;
    //
     ros::Subscriber gripperSubscriber;
    //
     bool gripperState = false;

    /*
     * @brief This is a private method of this class.
     *
     * @param
     *
     * @return
     */
     void gripperCallback(const std_msgs::Bool &);

  public:
    /*
     * @brief This is the constructor for the class.
     */
     KukaGripper();

    /*
     * @brief This is the first method of the class.
     */
     void gripperToggle(const bool &);

    /*
     * @brief This is the second method of the class.
     */
     bool getGripperState();

    /*
     * @brief This is the destructor for the class.
     */
     ~KukaGripper();

};
