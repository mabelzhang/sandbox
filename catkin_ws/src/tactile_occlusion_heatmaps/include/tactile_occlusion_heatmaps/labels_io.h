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

#include <Eigen/Core>


class LabelsIO
{
public:

  // Parameters:
  //   gpose: 1 x 7 row vector if orientation is parameterized by Quaternion
  static void write_label (const std::string path, const std::string obj_name,
    float quality, Eigen::VectorXf & gpose)
  {
    YAML::Emitter writer;

    // Example ouptut:
    //   obj_name: bar_clamp
    //   grasp_quality: 0.52
    //   gripper_pose: 
    // Ref: https://github.com/jbeder/yaml-cpp/wiki/How-To-Emit-YAML
    writer << YAML::BeginMap;
    writer << YAML::Key << "object_name";
    writer << YAML::Value << obj_name;

    writer << YAML::Key << "grasp_quality";
    writer << YAML::Value << quality;

    writer << YAML::Key << "gripper_pose";

      writer << YAML::BeginMap;
      writer << YAML::Key << "t_xyz";
      writer << YAML::Value << YAML::Flow << YAML::BeginSeq << gpose (0) << gpose (1) << gpose (2) << YAML::EndSeq;
 
      writer << YAML::Key << "q_xyz";
      writer << YAML::Value << YAML::Flow << YAML::BeginSeq << gpose (3) << gpose (4) << gpose (5) << YAML::EndSeq;
 
      writer << YAML::Key << "qw";
      writer << YAML::Value << gpose (6);

    writer << YAML::EndMap;

    // See what YAML string looks like
    //std::cerr << writer.c_str () << std::endl;

    // Write YAML string to file
    std::ofstream lbls_f;
    lbls_f.open (path);
    lbls_f << writer.c_str ();
    lbls_f.close ();
  }

};

#endif