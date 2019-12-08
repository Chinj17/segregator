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
