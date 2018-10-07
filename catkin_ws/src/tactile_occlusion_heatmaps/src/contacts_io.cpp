// Mabel Zhang
// 4 Oct 2018
//
// Read contacts files written by grasp_collection grasp_collection.py, which
//   contain positions of contacts expressed in object frame. Pass these
//   positions to occlusion_test.cpp to generate heatmaps based on contacts.
//

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


// Read paths defined in YAML file
// Counterpart of config_paths.py
class PathConfigYaml
{
private:

  std::string root_;

  std::string data_;
  std::string intrinsics_;
  std::string depth_range_;
  std::string grasps_;
  std::string contacts_;

  std::string renders_;
  std::string vis_;


public:

  PathConfigYaml (const std::string path)
  {
    if (! boost::filesystem::exists (path))
    {
      fprintf (stderr, "%sERROR: YAML file not found: %s%s\n", FAIL,
        path.c_str (), ENDC);
      return;
    }

    fprintf (stderr, "%sLoading paths YAML %s%s\n", OKCYAN, path.c_str (),
      ENDC);
    YAML::Node config = YAML::LoadFile (path);

    root_ = config ["root"].as <std::string> ();

    // Subdirectories under root
    join_paths (root_, config ["data"].as <std::string> (), data_);
    // Text files under data
    join_paths (data_, config ["intrinsics"].as <std::string> (), intrinsics_);
    join_paths (data_, config ["depth_range"].as <std::string> (),
      depth_range_);
    //Subdirectories under data
    join_paths (data_, config ["grasps"].as <std::string> (), grasps_);
    join_paths (data_, config ["contacts"].as <std::string> (), contacts_);

    // Subdirectories under root
    join_paths (root_, config ["renders"].as <std::string> (), renders_);
    join_paths (root_, config ["vis"].as <std::string> (), vis_);
  }

  void get_root (std::string & path)
  {
    path = root_;
  }

  void get_data_path (std::string & path)
  {
    path = data_;
  }

  void get_vis_path (std::string & path)
  {
    path = vis_;
  }

  void get_grasps_path (std::string & path)
  {
    path = grasps_;
  }

  void get_contacts_path (std::string & path)
  {
    path = contacts_;
  }
};


class ContactsYaml
{
private:

  std::vector <std::string> objects_;


public:

  ContactsYaml (const std::string path)
  {
    if (! boost::filesystem::exists (path))
    {
      fprintf (stderr, "%sERROR: YAML file not found: %s%s\n", FAIL,
        path.c_str (), ENDC);
      return;
    }

    fprintf (stderr, "%sLoading contacts YAML %s%s\n", OKCYAN, path.c_str (),
      ENDC);
    YAML::Node config = YAML::LoadFile (path);
    YAML::Node objects_node = config ["objects"];

    objects_.resize (objects_node.size ());
    for (std::size_t i = 0; i < objects_node.size (); i++)
    {
      objects_.push_back (objects_node [i].as <std::string> ());
      std::cerr << objects_ [i].c_str () << std::endl;
    }
  }

  void get_objects (std::vector <std::string> & rv)
  {
    rv = objects_;
  }
};



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


  // Get contacts file path
  std::string contacts_path;
  config.get_contacts_path (contacts_path);

  // Load each objects' contacts
  // First one is empty string
  for (int i = 0; i < contact_objs.size (); i ++)
  {
    // Read object contacts file
    std::string obj_path;
    join_paths (contacts_path, contact_objs [i], obj_path);

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
    Eigen::MatrixXd contacts_m = load_csv_to_Eigen <Eigen::MatrixXd> (
      obj_path);
    std::cout << contacts_m << std::endl;


    // Pass contacts onto occlusion_test.cpp. They go into "endpoints" variable
    //   in occlusion_test.cpp. Refactor main() in occlusion_test.cpp so it can
    //   accept 3D point inputs!

  }



  return 0;
}

