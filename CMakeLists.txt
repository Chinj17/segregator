cmake_minimum_required(VERSION 2.8.3)
project(segregator)

add_compile_options(-std=c++11)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge image_transport)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

include(CMakeToolsHelpers OPTIONAL)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

## The catkin_package macro generates cmake config files for your package
catkin_package(INCLUDE_DIRS include/segregator CATKIN_DEPENDS roscpp std_msgs)
## Build
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

## Specify additional locations of header files
include_directories(include/segregator ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(cam src/main.cpp src/KukaKinematics.cpp src/Gripper.cpp src/Detection.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cam ${OpenCV_LIBS} ${catkin_LIBRARIES})
