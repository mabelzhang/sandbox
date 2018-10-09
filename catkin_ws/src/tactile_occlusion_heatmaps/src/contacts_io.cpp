// Mabel Zhang
// 4 Oct 2018
//
// Read contacts files written by grasp_collection grasp_collection.py, which
//   contain positions of contacts expressed in object frame. Pass these
//   positions to occlusion_test.cpp to generate heatmaps based on contacts.
//

// C++
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <Eigen/Core>

// ROS
#include <ros/package.h>

// Custom
#include <util/io_util.h>
#include <util/eigen_util.h>
#include <util/ansi_colors.h>

// Local
#include <tactile_graspit_collection/contacts_io.h>
#include <tactile_graspit_collection/config_paths.h>  // PathConfigYaml


int main (int argc, char ** argv)
{

  // Read config file for paths

  std::string pkg_path = ros::package::getPath ("tactile_occlusion_heatmaps");

  std::string config_path;
  join_paths (pkg_path, "config/paths.yaml", config_path);

  PathConfigYaml config = PathConfigYaml (config_path);


  // Read config file for objects

  std::string contacts_config_path;
  join_paths (pkg_path, "config/contacts.yaml", contacts_config_path);

  std::vector <std::string> contact_objs;
  ContactsYaml contacts_config = ContactsYaml (contacts_config_path);
  contacts_config.get_objects (contact_objs);


  // Get contacts directory
  std::string contacts_dir;
  config.get_contacts_path (contacts_dir);

  // Load each objects' contacts
  // First one is empty string
  for (int i = 0; i < contact_objs.size (); i ++)
  {
    // Read object contacts file
    std::string obj_path;
    join_paths (contacts_dir, contact_objs [i], obj_path);

    std::vector <std::string> exts;
    splitext (obj_path, exts);
    if (exts.size () == 0)
    {
      fprintf (stderr, "Skipping. Not a file with extension\n");
      continue;
    }

    /* Don't need, `.` not reading any grasps
    // Read meta file for object contacts indexing for each grasp
    std::string obj_meta_path = exts [0] + "_meta" + exts [1];
    */


    // Read contacts csv file into Eigen Matrix
    fprintf (stderr, "Loading object contacts %s\n", obj_path.c_str ());
    Eigen::MatrixXf contacts_m = load_csv_to_Eigen <Eigen::MatrixXf> (
      obj_path);
    std::cout << contacts_m << std::endl;


    // TODO
    // Pass contacts onto occlusion_test.cpp. They go into "endpoints" variable
    //   in occlusion_test.cpp. Refactor main() in occlusion_test.cpp so it can
    //   accept 3D point inputs!

  }

  return 0;
}

