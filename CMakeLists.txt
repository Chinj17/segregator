cmake_minimum_required(VERSION 2.8.3)
project(segregator)

add_compile_options(-std=c++11)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

# We probably don't want this to run on every build.
option(COVERAGE "Generate Coverage Data" ON)

if (COVERAGE)
    include(CodeCoverage)
    setup_target_for_coverage(code_coverage ../devel/lib/segregator/allTests coverage)
    set(COVERAGE_SRCS src/main.cpp include/Detection.hpp include/KukaKinematics.hpp include/Gripper.hpp)

    SET(CMAKE_CXX_FLAGS "-g -O0 --coverage -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    SET(CMAKE_C_FLAGS "-g -O0 --coverage  -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
    SET(CMAKE_EXE_LINKER_FLAGS "--coverage -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
else()
    set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -g ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge image_transport geometry_msgs roscpp rospy std_msgs tf tf_conversions kdl_conversions)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
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

## Testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(allTests test/allTests.test test/main.cpp test/testDetection.cpp test/testGripper.cpp test/testKukaKinematics.cpp src/Detection.cpp src/Gripper.cpp src/KukaKinematics.cpp)
  target_link_libraries(allTests ${OpenCV_LIBS} ${catkin_LIBRARIES})
  add_dependencies(allTests ${catkin_EXPORTED_TARGETS})
endif()
