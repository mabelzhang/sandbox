// Mabel Zhang
// 17 Feb 2018
//
// ROS Gazebo Contact Sensor plugin to simulate TakkTile MEMS barometric
//   sensors, but with Boolean value only.
// Each sensor gets one instance of this plugin. 18 instances total.
//   Upon contact, publishes its finger number (0-2) and sensor number (0-17)
//   to robotiq_s_model_articulated_gazebo_plugins/Contact type message.
//
// Modified from my reflex_gazebo contact_sensor_plugin.cpp
// Modified from http://gazebosim.org/tutorials?cat=sensors&tut=contact_sensor
//
// gazebo::ContactPlugin API:
//   https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1ContactPlugin.html
// gazebo::sensors::ContactSensor API:
//   http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ContactSensor.html
//

// rosmsg
#include <robotiq_s_model_articulated_gazebo_plugins/Contact.h>

#include <robotiq_s_model_articulated_gazebo_plugins/contact_sensor_plugin.h>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactTutorialPlugin)

/////////////////////////////////////////////////
ContactTutorialPlugin::ContactTutorialPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactTutorialPlugin::~ContactTutorialPlugin()
{
  // Disconnect sensor, see if this deletes sensor from world properly when
  //   hand is removed from world using remove_model
  // API https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ContactSensor.html
  if (this -> parentSensor)
  {
    ROS_INFO ("Disconnecting contact sensor %s", this -> GetHandle ().c_str ());
    this -> parentSensor -> DisconnectUpdated (this -> updateConnection);
  }
}

/////////////////////////////////////////////////
// Save _sensor in member var. Tells Gazebo to call OnUpdate() when sensor gets
//   new values.
void ContactTutorialPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get the parent sensor.
  this->parentSensor =
    //boost::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactTutorialPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactTutorialPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);


  /////
  // ROS publisher
  /////

  // Follow rostopic name format in RobotiqHandPlugin.cpp
  contact_pub_ = nh_.advertise <robotiq_s_model_articulated_gazebo_plugins::Contact> (
    "/left_hand/contact", 5);


  // Find out which sensor on hand this instance of the plugin is hooked up to,
  //   assign a unique sensor number to this instance.

  // Get parent link name, parse the string to get the finger # and sensor #
  // SensorPlugin API: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1SensorPlugin.html
  // this->parentSensor ContactSensor API: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1ContactSensor.html

  // finger_1_sensor_6
  //std::string sensor_name = this -> parentSensor -> Name ();
  //printf ("Sensor name: %s\n", sensor_name.c_str ());

  // libcontact_tutorial_plugin.dylib
  //std::string file_name = this -> GetFilename ();
  //printf ("File name: %s\n", file_name.c_str ());


  // 0 to 2
  int fin_num = 0;
  // 0 to 17
  int sen_num = 0;


  /*
  // <plugin name="...">
  // Defined in robotiq_hand_wtactile.gazebo
  std::string hdl_name = this -> GetHandle ();
  printf ("Handle name: %s\n", hdl_name.c_str ());

  // Didn't want to change finger_prefix in robotiq_hand_wtactile.gazebo to
  //   numerical names. No time for that, would need to change all names in
  //   URDF.
  // l_finger_<0-2>_sensor_<0-17>_plugin
  // This corresponds to naming in my robotiq_takktile takktile_consts.py
  // 0 for Finger A (finger_middle)
  // 1 for Finger B (finger_2)
  // 2 for Finger C (finger_1)
  std::string finger_prefix = "l_finger_%d_";

  // Defined in robotiq_hand_wtactile.gazebo
  // Grab this from the URDF file, urdf/full_reflex_model.gazebo
  // ATTENTION if change name there, need to change here accordingly, otherwise
  //   sensors won't get assigned the correct number, and you won't get the
  //   correct /reflex_hand message that tells you which sensor is activated!
  std::string name_format = finger_prefix + "sensor_%d_plugin";

  sscanf (hdl_name.c_str (), name_format.c_str (), &fin_num, &sen_num);
  */


  std::string sensor_name = this -> parentSensor -> Name ();

  int sen_str_i = 0;

  // Never mind. Still would need to find sensor number, which means parsing
  //   even more.
  // l_finger_<1 | 2 | middle>_sensor_<0-17>
  // This corresponds to naming in my robotiq_takktile takktile_consts.py
  // 0 for Finger A (finger_middle)
  // 1 for Finger B (finger_2)
  // 2 for Finger C (finger_1)
  if (sensor_name.find ("l_finger_1_") != std::string::npos)
  {
    fin_num = 2;
  }

  if (sensor_name.find ("l_finger_2_") != std::string::npos)
  {
    fin_num = 1;
  }

  if (sensor_name.find ("l_finger_middle_") != std::string::npos)
  {
    fin_num = 0;
  }

  sen_str_i = sensor_name.find_last_of ("_");
  sen_num = std::stoi (sensor_name.substr (sen_str_i+1));


  printf ("Parsed (definition: finger_1 = 2, finger_2 = 1, finger_middle = 0): %d %d\n", fin_num, sen_num);

  contact_msg_.fin_num = fin_num;
  contact_msg_.sen_num = sen_num;


  ROS_INFO ("Gazebo contact sensor plugin initialized for finger %d sensor %d, sensor name %s",
    fin_num, sen_num, this -> parentSensor -> Name ().c_str ());
  ROS_INFO ("Collision name: %s", this->parentSensor->GetCollisionName (0).c_str ());
}

/////////////////////////////////////////////////
void ContactTutorialPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  //for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  //{
    // Valuable info. Skipping printing because of clutter. Turn on if debugging
    // Print the two bodies in collision
    // contacts.contact(i) is physics::Contact type
    //   API https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1physics_1_1Contact.html
    //std::cout << "Collision between[" << LOW_BLUE << contacts.contact(i).collision1() << ENDC
    //          << "] and [" << OKCYAN << contacts.contact(i).collision2() << ENDC << "]\n";

    // Too much clutter
    // Print low level physics information
    /*
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      std::cout << j << "  Position:"
                << contacts.contact(i).position(j).x() << " "
                << contacts.contact(i).position(j).y() << " "
                << contacts.contact(i).position(j).z() << "\n";
      std::cout << "   Normal:"
                << contacts.contact(i).normal(j).x() << " "
                << contacts.contact(i).normal(j).y() << " "
                << contacts.contact(i).normal(j).z() << "\n";
      std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
    }
    */
  //}

  // NOTE: If you don't get any contacts, check that your
  //   <sensor><contact><collision> name matches the actual <link><collision>
  //   name in the SDF file - not the URDF file, SDF automatically collapsed
  //   fixed joints and appends integer suffixes to collision names. It
  //   ignores whatever name you define in <link><collision> of the URDF!
  //   To see the SDF file:
  //   $ xacro file.xacro > file.urdf
  //   $ gz sdf --print file.urdf > file.sdf
  //   Then manually inspect file.sdf to make sure the sensor collision name
  //   matches the actual link collision name. If it doesn't match, you won't
  //   get any contacts!
  // If this sensor got any contacts at all, publish this sensor's identity to
  //   the topic, so subscribers know this sensor was activated
  if (contacts.contact_size () > 0)
  {
    //fprintf (stderr, "Got contact on finger %d sensor %d\n",
    //  contact_msg_.fin_num, contact_msg_.sen_num);

    contact_msg_.header.stamp = ros::Time::now ();
    contact_pub_.publish (contact_msg_);
  }
}

