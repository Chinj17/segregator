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
 * @brief This is the header file of the Kuka Gripper class
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi - Driver
 * @author Kamakshi Jain - Navigator
 * @author Sayan Brahma - Design Keeper
 * @date 12-7-2019
 */

#include "Detection.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


Detection::Detection(KukaKinematics & ku, const bool & display) : imgT(n),
                                                kuka(ku), dispImg(display) {

    imageSubscriber = imgT.subscribe("/camera/image_raw", 1,
                                                    &Detection::readImg, this);

    if (dispImg) {
      cv::namedWindow(OPENCV_WINDOW);
    }

}

std::string Detection::colorThresholder(const KukaKinematics::States & pos) {
    auto posInd = static_cast<int>(pos);
    cv::Vec3b slab;

    // Define pixel for the corresponding slab
    if (posInd == 1) {
        slab = cv_ptr->image.at<cv::Vec3b>(179, 185);
    }
    else if (posInd == 2) {
        slab = cv_ptr->image.at<cv::Vec3b>(57, 187);
    } else {
        return "";
    }
    // Detect the color of the slab
    if ((slab.val[0] == 255) && (slab.val[1] != 255) && (slab.val[2] != 255)) {
        return "blue";
    } else if ((slab.val[1] == 255) && (slab.val[0] != 255) && (slab.val[2] != 255)) {
        return "green";
    } else if ((slab.val[2] == 255) && (slab.val[1] != 255) && (slab.val[2] != 255)) {
        return "red";
    } else {
        return "";
    }
}

void Detection::readImg(const sensor_msgs::ImageConstPtr & msg) {
    try {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        return;
	}
    	if (dispImg)
       		cv::imshow(OPENCV_WINDOW, cv_ptr->image);
}

Detection::~Detection() {
    if (dispImg)
       cv::destroyWindow(OPENCV_WINDOW);
}
