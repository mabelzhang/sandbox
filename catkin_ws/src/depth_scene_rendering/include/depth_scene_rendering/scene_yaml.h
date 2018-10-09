#ifndef _SCENE_YAML_H_
#define _SCENE_YAML_H_

// Mabel Zhang
// 9 Oct 2018
//
// Classes to read YAML config files
//

#include <boost/filesystem.hpp>

#include <yaml-cpp/yaml.h>


// YAML file outputted by scene_generation.py, manually as plain-text, `.` run
//   in Blender, which does not have YAML library.
class ScenesYaml
{
private:

  std::vector <std::string> objects_;

  // Root node of file
  YAML::Node objects_node;


public:

  ScenesYaml (const std::string path)
  {
    if (! boost::filesystem::exists (path))
    {
      fprintf (stderr, "%sERROR: YAML file not found: %s%s\n", FAIL,
        path.c_str (), ENDC);
      return;
    }

    fprintf (stderr, "%sLoading scenes YAML %s%s\n", OKCYAN, path.c_str (),
      ENDC);
    YAML::Node config = YAML::LoadFile (path);
    objects_node = config ["objects"];

    objects_.resize (objects_node.size ());
    fprintf (stderr, "YAML file contains %ld objects\n", objects_node.size ());
    // Loop through each object
    for (std::size_t i = 0; i < objects_node.size (); i++)
    {
      //std::cerr << objects_node [i] << std::endl;
      objects_ [i] = objects_node [i] ["object"].as <std::string> ();
      std::cerr << "Object " << objects_ [i].c_str () << std::endl;
    }
  }

  void get_objects (std::vector <std::string> & rv)
  {
    rv = objects_;
  }

  int get_n_objects ()
  {
    return objects_.size ();
  }

  const std::string get_object_name (int i)
  {
    return objects_.at (i);
  }  

  // Get all the pcd paths for the specified object
  void get_scenes (int obj_i, std::vector <std::string> & scenes)
  {
    scenes.clear ();

    fprintf (stderr, "Object contains %ld scenes\n", objects_node [obj_i].size ());

    // Loop through each scene
    for (std::size_t j = 0; j < objects_node [obj_i] ["scenes"].size (); j++)
    {
      scenes.push_back (objects_node [obj_i] ["scenes"] [j].as <std::string> ());
    }
  }

};

#endif
