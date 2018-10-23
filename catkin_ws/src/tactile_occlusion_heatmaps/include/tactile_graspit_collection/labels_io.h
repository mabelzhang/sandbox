#ifndef _LABELS_IO_H_
#define _LABELS_IO_H_

// Mabel Zhang
// 22 Oct 2018
//
// Output label for each object scene. Each scene is one example for predictor.
//

#include <iostream>
#include <fstream>

#include <yaml-cpp/yaml.h>


class LabelsIO
{
public:


  static void write_label (const std::string path, const std::string obj_name,
    float quality)
  {
    YAML::Emitter writer;

    // Example ouptut:
    //   obj_name: bar_clamp
    //   grasp_quality: 0.52
    // Ref: https://github.com/jbeder/yaml-cpp/wiki/How-To-Emit-YAML
    writer << YAML::BeginMap;
    writer << YAML::Key << "object_name";
    writer << YAML::Value << obj_name;
    writer << YAML::Key << "grasp_quality";
    writer << YAML::Value << quality;
    writer << YAML::EndMap;

    //std::cerr << writer.c_str () << std::endl;

    // Write YAML string to file
    std::ofstream lbls_f;
    lbls_f.open (path);
    lbls_f << writer.c_str ();
    lbls_f.close ();
  }


};

#endif
