// Mabel Zhang
// 5 Sep 2018
//
// Load .pcd scene files outputted from BlenSor, by depth_scene_rendering
//   package.
// Given a contact point, test whether the point is in front of, or behind,
//   depth camera's reach.
//
// Usage:
//   $ rosrun tactile_graspit_collection occlusion_test
//

// C++
#include <fstream>
#include <string>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Custom packages
#include <util/io_util.h>  // join_paths ()
#include <util/pcd_util.h>
#include <util/pcl_raytrace_util.h>



int main (int argc, char ** argv)
{
  ros::init (argc, argv, "occlusion_test");
  ros::NodeHandle nh;


  // Get path of package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  // Text file with list of .pcd scene names
  std::string noisy_scene_list_path = "";
  join_paths (pkg_path, "config/noisy_scenes.txt", noisy_scene_list_path);
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
    RayTracer raytracer = RayTracer (cloud_ptr, octree_res, true, &nh);

    printf ("Testing ray-tracing...\n");
    // Origin of ray is always from camera center, 0 0 0.
    Eigen::Vector3f origin (0, 0, 0);
    // 1 m along z of camera frame, i.e. straight out of and normal to image
    //   plane.
    // TODO: Seems like Blender decides camera faces -z. So will shoot to -z.
    //   Either that, or BlenSor pcd is in world frame. Check if that is the
    //   case, by moving object elsewhere. Fix it so that the pcd is in camera
    //   frame!
    Eigen::Vector3f endpoint (0, 0, -1);
    bool occluded = raytracer.raytrace_occlusion_test (origin, endpoint);
    printf ("Occluded? %s\n", occluded ? "true" : "false");


  }



  return 0;
}

