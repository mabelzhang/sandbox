// Mabel Zhang
// 5 Sep 2018
//
// Load .pcd scene files outputted from BlenSor, by depth_scene_rendering
//   package.
// Given a contact point, test whether the point is in front of, or behind,
//   depth camera's reach.
//

// C++
#include <fstream>
#include <string>
#include <stdio.h>
//#include <experimental/filesystem>  // C++17

// ROS
#include <ros/package.h>

// Custom packages
#include <util/io_util.h>
#include <util/pcd_util.h>
#include <util/pcl_raytrace_util.h>



int main (int argc, char ** argv)
{
  // Need C++17. Doesn't work with linker, not sure why.
  //std::string this_dir = std::experimental::filesystem::current_path ();
  // Can use boost version instead
  //boost::filesystem::path this_dir (boost:filesystem::current_path ());


  // Get path of this package
  std::string pkg_path = ros::package::getPath ("tactile_graspit_collection");

  // Text file with list of .pcd scene names
  std::string noisy_scene_list_path = "";
  join_paths (pkg_path, "../depth_scene_rendering/config/noisy_scenes.txt",
    noisy_scene_list_path);
  std::ifstream noisy_scene_list_f (noisy_scene_list_path.c_str ());

  // Octree resolution
  float octree_res = 128.0f;

  // Read text file line by line
  std::string scene_name = "";
  while (std::getline (noisy_scene_list_f, scene_name))
  {
    // Instantiate cloud
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr =
      pcl::PointCloud <pcl::PointXYZ>::Ptr (
        new pcl::PointCloud <pcl::PointXYZ> ());
 
    // Load scene cloud
    load_cloud_file (scene_name, cloud_ptr);
    printf ("Cloud size: %ld points\n", cloud_ptr->size ());
    printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");

    // Make octree to hold point cloud, for raytrace test
    // Ref: http://pointclouds.org/documentation/tutorials/octree.php
    RayTracer raytracer = RayTracer (cloud_ptr, octree_res);

    printf ("Testing ray-tracing...\n");
    Eigen::Vector3f origin (-1, 0, 0);
    Eigen::Vector3f direction (1, 0, 0);
    Eigen::Vector3f point (0, 0, 0);
    raytracer.raytrace_occlusion_test (origin, direction, point);


  }



  return 0;
}

