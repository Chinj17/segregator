/**
 * @file CameraPlugins.hpp
 * Copyright [2019] sayan brahma - driver, chinmay joshi - navigator
 * @brief This is the declaration of the CameraPlugin class
 */

#ifndef INCLUDE_CAMERAPLUGIN_HPP
#define INCLUDE_CAMERAPLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

/**
 * @brief A plugin to control the camera sensor.
 */
class CameraPlugin : public SensorPlugin {
 public:
  /**
   * @brief This is the constructor for the class.
   */
  CameraPlugin();

  /**
   * @brief This is the first method of the class. It is a load function
   *        which is called by Gazebo when the plugin is inserted into
   *        simulation.
   *
   * @param The first parameter to this method is a pointer to the sensor
   *        that this plugin is attached to.
   * @param The second parameter to this method is a pointer to the plugin's
   *        SDF element.
   */
  virtual void Load(sensors::SensorPtr, sdf::ElementPtr);
};

#endif  // INCLUDE_CAMERAPLUGIN_HPP

