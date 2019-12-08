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
    else {
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
}
