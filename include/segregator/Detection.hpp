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
 * @file Detection.hpp
 * @brief This is the header file of the Kuka Detection class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */

#ifndef INCLUDE_SEGREGATOR_DETECTION_HPP_
#define INCLUDE_SEGREGATOR_DETECTION_HPP_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "KukaKinematics.hpp"

/*
 * @brief Detection is a class used for working with the camera in the world
 */
class Detection {
 private:
    // object of KukaKinematics class
    KukaKinematics & kuka;
    // check to display image or not
    bool dispImg = false;
    // ROS node handle
    ros::NodeHandle n;
    // variable to read image data
    cv_bridge::CvImagePtr cv_ptr;
    // Subscriber to image transport
    image_transport::Subscriber imageSubscriber;
    // cv_bridge variable for converting sensor_imgs to readable data
    image_transport::ImageTransport imgT;
    // variable to name the window
    const std::string OPENCV_WINDOW = "Image Window";

 public:
    /*
     * @brief This is the constructor for the class.
     *
     * @param Creates a subscriber for the image data.
     * @param Denotes whether or not to display image.
     *
     * @return Does not return anything.
     */
    explicit Detection(KukaKinematics &, const bool &);

    /*
     * @brief This is the first method of the class. It detects the position
     *        index of a particularly colored object.
     *
     * @param This function takes colour of the object as input.
     *
     * @return This function returns the position index for that object.
     */
    std::string colorThresholder(const KukaKinematics::States &);

    /*
     * @brief This is the second method of the class. Reads the Image
     *        captured by camera sensor.
     *
     * @param Input is the message being published by camera_raw.
     *
     * @return This method does not return anything.
     *
    */
    void readImg(const sensor_msgs::ImageConstPtr&);

    /*
     * @brief This is the destructor for the class
     *
     * @param No inputs as it is the desctructor
     *
     * @return This is the destructor so it does not return anything
     */
    ~Detection();
};

#endif  // INCLUDE_SEGREGATOR_DETECTION_HPP_
