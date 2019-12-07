/**
 * @file CameraPlugin.cpp
 * Copyright [2019] sayan brahma - Driver, chinmay - Navigator
 * @brief This is the implementation of the CameraPlugin class
 */

#include "CameraPlugin.hpp"

// @brief This is the constructor of the class
CameraPlugin::CameraPlugin() {
  std::cout << "Camera Plugin has been initialized." << std::endl;
}

// @brief This is the first method of the class. It is called by Gazebo when the
// @brief plugin is inserted into simulation
void CameraPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
  // Just output a message for now
  std::cout << "The camera plugin is attached to sensor: " << sensor->GetName()
      << std::endl;
}

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

