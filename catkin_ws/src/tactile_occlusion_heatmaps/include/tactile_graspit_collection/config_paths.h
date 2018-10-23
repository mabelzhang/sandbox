// Mabel Zhang
// 4 Oct 2018
//
// C++ counterpart of grasp_collection config_paths.py
//
// Paths are configured in YAML by hand in
//   tactile_occlusion_heatmaps/config/paths.yaml
//

#ifndef _CONFIG_PATHS_H_
#define _CONFIG_PATHS_H_

// Read paths defined in YAML file
class PathConfigYaml
{
private:

  std::string root_;

  std::string data_;
  std::string intrinsics_;
  std::string depth_range_;
  std::string grasps_;
  std::string contacts_;
  std::string quals_;

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
    // Subdirectories under data
    join_paths (data_, config ["grasps"].as <std::string> (), grasps_);
    join_paths (data_, config ["contacts"].as <std::string> (), contacts_);
    join_paths (data_, config ["quals"].as <std::string> (), quals_);

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

  void get_quals_path (std::string & path)
  {
    path = quals_;
  }
};

#endif
