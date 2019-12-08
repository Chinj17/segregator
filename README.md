# segregator - ENPM808X FINAL PROJECT
[![Build Status](https://travis-ci.org/sbrahma0/segregator.svg?branch=Implementation)](https://travis-ci.org/sbrahma0/segregator)
[![Coverage Status](https://coveralls.io/repos/github/kamakshijain/segregator/badge.svg?branch=master)](https://coveralls.io/github/kamakshijain/segregator?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---
# Overview
In today’s fast-paced production environments, picking and packing operations demand uninterrupted speed, reliability, inspection, sorting, accuracy and dexterity from human operator. To increase productivity and ease of implementation, we will be developing software for a Kuka robotic manipulator that can segregate different colored packages and place them on preassigned belts/cases. This will not only lower costs versus manual labor, but also saves valuable production time for Acme Robotics.
We will be defining the zeroth configuration of the Kuka robot. An overhead camera will detect the color of the package that can be modified by the user. Local coordinate of the object will be published w.r.t the manipulator’s location. Eventually the vacuum gripper of the manipulator will be brought to the object location through an optimal trajectory, dodging obstacles (static) if any. The gripper will be activated to pick and place the object in the desired station. Category - “Material handling robot”.

## Demonstration

## Video File

## About the authors
Here is a little information about the authors:

* Kamakshi Jain
* Sayan Brahma
* Chinmay Joshi <br>
I am a graduate student doing my Master's in Robotics at the University of Maryland. My Bachelor's degree was in Electronics and Instrumentation Engineering from Vellore Institute of Technology, India. I would like to work in the field of Robotics with a focus on either Computer Vision or Machine Learning.

### Technologies to be used:

   * Ubuntu 16.04
   * ROS Kinetic
   * OpenCV
   * Gazebo for simulation
   * Kuka Manipulator Model, vacuum gripper model, camera module

## AIP Logs
[AIP Spreadsheet](https://docs.google.com/spreadsheets/d/1l3zZY-S-sCEj8x_SvJREo7-diR4zwI-w_J22i3sZJyI/edit?usp=sharing)- This contains our product backlog, iteration backlog, and work log.
## Sprint doc
[Sprint Planing](https://docs.google.com/document/d/1x5kZCbR9iNZZeu6fHsHf5Rvu3TCuqFMhLQ0rkCWsUQ4/edit?usp=sharing) - This contains the google doc with sprint plans and problems faced.

## LICENSE
### Copyright <2019> <Kamakshi Jain> <Sayan Brahma> <Chinmay Joshi>
```
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
## Dependencies

* To install ROS Kinetic in Ubuntu 16.04, follow the steps in this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu).

* To install catkin, follow the installation steps in this [link](http://wiki.ros.org/catkin).

### opencv
Install OpenCV 3.3.0 using the following commands:

Install OpenCV Dependencies
```
sudo apt-get install build-essential checkinstall cmake pkg-config yasm gfortran git
sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev
sudo apt-get install libtiff5-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev
sudo apt-get install libxine2-dev libv4l-dev
sudo apt-get install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
sudo apt-get install libqt4-dev libgtk2.0-dev libtbb-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install libvorbis-dev libxvidcore-dev
sudo apt-get install libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install x264 v4l-utils
```
Download and Compile OpenCV
```
git clone https://github.com/opencv/opencv.git
cd opencv
git checkout 3.3.0
cd ..
git clone https://github.com/opencv/opencv_contrib.git
cd opencv_contrib
git checkout 3.3.0
cd ..
cd opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_QT=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
```


### Ros Control Dependencies (REQUIRED)
Make sure you have these packages installed in the environment:

* ros-kinetic-velocity-controllers
* ros-kinetic-ros-control
* ros-kinetic-position-controllers
* ros-kinetic-joint-state-controller
* ros-kinetic-joint-trajectory-controller
If not, type:

```
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control ros-kinetic-position-controllers ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller

sudo apt-get install ros-kinetic-moveit

sudo apt-get install ros-kinetic-trajectory* //not helpful

sudo apt-get install ros-kinetic-moveit* //not helpful

sudo apt-get install ros-kinetic-joints* //not helpful

sudo apt install ros-kinetic-gazebo-ros-control
```

## Build Instructions
To build this code in a catkin workspace:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/kamakshijain/segregator.git
cd ..
catkin_make
```
Note, that if you do not have a catkin workspace, then to build this code use the following commands:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/kamakshijain/segregator.git
cd ..
catkin_make
```
### Installation of additional packages
In your catkin workspace directory (or create a new one using the above instructions)
```
git clone https://github.com/kamakshijain/iiwa_stack.git
cd ..
catkin_make
source devel/setup.bash
```
## Running the Demo using Launch File

To run the demo, a launch file has been created. This launch file loads the Gazebo environment and runs the cam node to detect the objects on the table and segregate them into bins based on their color.

**Note: This is an ongoing project and the following instructions may not run yet.**


After following the build instructions, to run the demo, launch the code using the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch segregator kuka_fwd.launch
```

## Error checks
**cppcheck**
```
```

**cpplint**
```
```
