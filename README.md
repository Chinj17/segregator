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


### Ros Control Dependencies (REQUIRED)
Make sure you have these packages installed in the environment:

* ros-kinetic-velocity-controllers
* ros-kinetic-ros-control
* ros-kinetic-position-controllers
* ros-kinetic-joint-state-controller
* ros-kinetic-joint-trajectory-controller
* ros-kinetic-gazebo-ros-control

If not installed, type:

```
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control
sudo apt-get install ros-kinetic-position-controllers ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-joint-trajectory-controller ros-kinetic-moveit
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

After following the build instructions, to run the demo, launch the code using the following commands:
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch segregator segregator.launch
```

To be in line with the need of the project we have added various user inputs which basically gives different color slabs for the robot to pick and place. Various user inputs can be launched using the following commands:
```
roslaunch segregator segregator.launch Color:=BB
roslaunch segregator segregator.launch Color:=BR
roslaunch segregator segregator.launch Color:=GB
roslaunch segregator segregator.launch Color:=RB
roslaunch segregator segregator.launch Color:=RG
roslaunch segregator segregator.launch Color:=RR
roslaunch segregator segregator.launch Color:=YG
```
## Record Bag File
A ROS bag file is used to record all the topic and messages being published in the terminal. After following the build instructions, run the following commands to record the bag file:

```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch segregator segregator.launch record:=enable
```

The bag file can be found in the results folder as that is its default storing location.

**Inspecting the bag file** <br>
To get information on the generated bag file, run the following:

```
cd ~/catkin_ws/src/segregator/results/
rosbag info kuka.bag
```

**Playing the bag file** <br>
To play the bag file, ROS master has to be running. In a new terminal type the following:
```
roscore
```
In a new terminal, run the following command:
```
cd ~/catkin_ws/src/segregator/results/
rosbag play kuka.bag
```

**Playing the bag file to Observe motion** <br>
As stated above, you can simply play the bag file. But, to get a better understanding of what is actually happening you might need to see the motion being generated by the robot. To do so, follow the steps below. After following the build instructions, launch the Gazebo world by running the following commands:
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch
```

To only run Gazebo, and not Rviz, run
```
source ~/catkin_ws/devel/setup.bash
roslaunch iiwa_moveit moveit_planning_execution.launch rviz:=false
```

Now once the above steps are done, in a new terminal type the following
```
cd ~/catkin_ws/src/segregator/results/
rosbag play kuka.bag
```

## Run Tests

## Error checks
**cppcheck** <br>
To run cppcheck in Terminal, run the following commands:
```
cd <path to repository>
cppcheck --std=c++11 -I include/ --suppress=missingIncludeSystem $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
```

**cpplint** <br>
To check Google C++ Style formatting in Terminal, run the following commands:
```
cd <path to repository>
cpplint $(find . -name \*.cpp -or -name \*.hpp | grep -vE -e "^./docs/" -e "^./launch/" -e "^./results/" -e "^./UML/" -e "./world/")
```

## Known Issues/Bugs

## Generating Doxygen Documentation
To install doxygen run the following command:
```
sudo apt install doxygen
````
Now, to generate doxygen documentation, run the following commands:
```
cd ~/catkin_ws/src/segregator/
doxygen doxconfig
```
Doxygen files will be generated to /docs folder. To view them in a browser, run the following commands:
```
cd docs/html
firefox index.html
``
