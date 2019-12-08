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
#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
 * @brief Detection is a class used for working with the camera in the world
 */
class Detection {
 private:
    //
    KukaKinematics & kuka;
    //
    bool dispImg = false;
    //
    ros::NodeHandle n;
    //
    cv_bridge::CvImagePtr cv_ptr;
    //
    image_transport::Subscriber imageSubscriber;
    //
    image_transport::ImageTransport imgT;
    //
    const std::string OPENCV_WINDOW = "Image Window";

 public:
    /*
     * @brief This is the constructor for the class
     */
    explicit Detection(KukaKinematics &, const bool &);
    /*
     * @brief This is the first method of the class. It detects the position
     *        index of a particularly colored object.
     *
     * @param This function takes colour of the object as input.
     *
     * @result This function returns the position index for that object.
     */
    std::string colorThresholder(const KukaKinematics::States &);
    /*
     * @brief This is the second method of the class.
     * @param
     * @return
     *
    */
    void readImg(const sensor_msgs::ImageConstPtr&);


    /*
     * @brief This is the destructor for the class
     */
    ~Detection();
};

#endif  // INCLUDE_SEGREGATOR_DETECTION_HPP_
