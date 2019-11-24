# C++ Boilerplate
[![Build Status](https://travis-ci.org/kamakshijain/segregator.svg?branch=master)](https://travis-ci.org/kamakshijain/segregator)
[![Coverage Status](https://coveralls.io/repos/github/kamakshijain/segregator/badge.svg?branch=master)](https://coveralls.io/github/kamakshijain/segregator?branch=master)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---
# Overview
In today’s fast-paced production environments, picking and packing operations demand uninterrupted speed, reliability, inspection, sorting, accuracy and dexterity from human operator. To increase productivity and ease of implementation, we will be developing software for a Kuka robotic manipulator that can segregate different colored packages and place them on preassigned belts/cases. This will not only lower costs versus manual labor, but also saves valuable production time for Acme Robotics. 
We will be defining the zeroth configuration of the Kuka robot. An overhead camera will detect the color of the package that can be modified by the user. Local coordinate of the object will be published w.r.t the manipulator’s location. Eventually the vacuum gripper of the manipulator will be brought to the object location through an optimal trajectory, dodging obstacles (static) if any. The gripper will be activated to pick and place the object in the desired station. Category - “Material handling robot”.

### Technologies to be used: 

   * Ubuntu 16.04 
   * ROS Kinetic 
   * OpenCV 
   * Gazebo for simulation 
   * Kuka Manipulator Model, vacuum gripper model, camera module 

## AIP Logs
[AIP Spreadsheet](https://docs.google.com/spreadsheets/d/1l3zZY-S-sCEj8x_SvJREo7-diR4zwI-w_J22i3sZJyI/edit?usp=sharing)- This contains our product backlog, iteration backlog, and work log.
## Sprint doc
[Sprint Planing](https://docs.google.com/document/d/1q1fZa8T8o8WiPjqkdCYfT_voSZSvZKb0-bGTY7lCoBc/edit?usp=sharing) - This contains the google doc with sprint plans and problems faced.

## LICENSE
### Copyright <2019> <Kamakshi Jain> <Sayan Brahma> <Chinmay Joshi>
```
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

