// Mabel Zhang
// 4 Oct 2018
//
// C++ counterpart of config_paths.py in depth_scene_rendering and
//   grasp_collection packages.
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
  //std::string intrinsics_;
  //std::string depth_range_;

  std::string renders_;
  std::string heatmaps_;
  std::string grasps_;
  std::string contacts_;
  std::string energies_;

  std::string energy_abbrev_;

  std::string test_renders_;
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

    // Text files under data/renders
    //join_paths (renders_, config ["intrinsics"].as <std::string> (), intrinsics_);
    //join_paths (renders_, config ["depth_range"].as <std::string> (),
    //  depth_range_);

    // Subdirectories under data
    join_paths (data_, config ["renders"].as <std::string> (), renders_);
    join_paths (data_, config ["heatmaps"].as <std::string> (), heatmaps_);
    join_paths (data_, config ["grasps"].as <std::string> (), grasps_);
    join_paths (data_, config ["contacts"].as <std::string> (), contacts_);
    join_paths (data_, config ["energies"].as <std::string> (), energies_);

    // Subdirectories under root
    join_paths (root_, config ["test_renders"].as <std::string> (),
      test_renders_);
    join_paths (root_, config ["vis"].as <std::string> (), vis_);

    energy_abbrev_ = config ["energy_abbrev"].as <std::string> ();
  }

  void get_data_root (std::string & path)
  {
    path = root_;
  }

  void get_data_path (std::string & path)
  {
    path = data_;
  }

  void get_renders_path (std::string & path)
  {
    path = renders_;
  }

  void get_heatmaps_path (std::string & path)
  {
    path = heatmaps_;
  }

  void get_grasps_path (std::string & path)
  {
    path = grasps_;
  }

  void get_contacts_path (std::string & path)
  {
    path = contacts_;
  }

  void get_energies_path (std::string & path)
  {
    path = energies_;
  }

  void get_energy_abbrev (std::string & abbrev)
  {
    abbrev = energy_abbrev_;
  }

  void get_vis_path (std::string & path)
  {
    path = vis_;
  }

};

#endif
