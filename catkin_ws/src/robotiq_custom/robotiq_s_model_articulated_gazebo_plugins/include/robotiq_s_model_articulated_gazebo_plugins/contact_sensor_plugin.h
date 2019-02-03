// Mabel Zhang
// 17 Feb 2018
//
// Modified from my reflex_gazebo contact_sensor_plugin.h
// Modified from http://gazebosim.org/tutorials?cat=sensors&tut=contact_sensor&ver=1.9

#ifndef _GAZEBO_CONTACT_PLUGIN_HH_
#define _GAZEBO_CONTACT_PLUGIN_HH_

#include <string>

// ROS Gazebo plugin
#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>

#include <gazebo/sensors/sensors.hh>

#include <std_msgs/Int32.h>

// My packages
#include <util/ansi_colors.h>

// My rosmsg
#include <robotiq_s_model_articulated_gazebo_plugins/Contact.h>


namespace gazebo
{
  class ContactTutorialPlugin : public SensorPlugin
  {
    public:
 
      ContactTutorialPlugin ();
      virtual ~ContactTutorialPlugin ();
 
      // I thnk Gazebo calls this, upon reading your SDF <plugin> tag. It passes
      //   in the _sensor and _sdf. The function then uses the _sensor passed in
      //   to populates the parentSensor member field. Then you can use
      //   that var to do whatever you want.
      //   It also tells Gazebo to call the OnUpdate() function, when sensor is
      //   updated. That is where you do whatever you want.
      /// \brief Load the sensor plugin.
      /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
      /// \param[in] _sdf SDF element that describes the plugin.
      virtual void Load (sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
 
    private:
 
      /// \brief Pointer to the contact sensor
      sensors::ContactSensorPtr parentSensor;
 
      /// \brief Connection that maintains a link between the contact sensor's
      /// updated signal and the OnUpdate callback.
      event::ConnectionPtr updateConnection;
 
 
      // ROS
 
      // Publish contact info on rostopic
      ros::NodeHandle nh_;
      ros::Publisher contact_pub_;
 
      robotiq_s_model_articulated_gazebo_plugins::Contact contact_msg_;
 
 
      /// \brief Callback that receives the contact sensor's update signal.
      virtual void OnUpdate ();

  };
}
#endif
