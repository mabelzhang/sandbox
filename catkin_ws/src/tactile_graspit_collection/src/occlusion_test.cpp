// Mabel Zhang
// 5 Sep 2018
//
// Load .pcd scene files outputted from BlenSor, by depth_scene_rendering
//   scene_generation.py.
// Given a contact point, test whether the point is in front of, or behind,
//   depth camera's reach (point cloud). If the point is behind at least one
//   point in the point cloud, it is considered occluded (behind). Otherwise,
//   (ray is in free space), it is unoccluded.
//
// Usage:
//   $ rosrun tactile_graspit_collection occlusion_test
//

// C++
#include <fstream>
#include <string>
#include <stdio.h>
#include <time.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Custom packages
#include <util/io_util.h>  // join_paths ()
#include <util/pcd_util.h>
#include <util/pcl_raytrace_util.h>  // RayTracer


// Separate contact points into visible and occluded channels.
class OcclusionSeparation
{
private:



public:

  OcclusionSeparation ()
  {

  }

  // Separate b
  // If ray is always in free space, it is behind (occluded)???
  // If ray hits the camera point cloud, it is in front? No, it has to be
  //   BEHIND camera point cloud... So it should be the other way around.
  //   Green = in front (visible), red = behind (occluded, goes through a pt
  //   in point cloud).
  void separate ()
  {


  }


};



int main (int argc, char ** argv)
{
  ros::init (argc, argv, "occlusion_test");
  ros::NodeHandle nh;

  // Random seed
  srand (time (NULL));


  // Get path of package
  std::string pkg_path = ros::package::getPath ("depth_scene_rendering");

  // Text file with list of .pcd scene names
  std::string noisy_scene_list_path = "";
  join_paths (pkg_path, "config/scenes_noisy.txt", noisy_scene_list_path);
  std::ifstream noisy_scene_list_f (noisy_scene_list_path.c_str ());

  // Octree resolution, in meters
  // 0.005 too large, some endpoints judged as occluded should be in front.
  // TODO: 0.002 still happens. Find out why, maybe raytrace_occlusion_test()
  //   doesn't decide correctly.
  float octree_res = 0.002;

  // Noise of randomly generated endpoint, in meters
  float noise_res = 0.01;

  // Read text file line by line. Each line is the path to a .pcd scene file
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
    // Blender camera faces -z. So will shoot to -z.
    //Eigen::Vector3f endpoint (0, 0, -1);

    // Generate a number between 1 and 10
    int nPts = rand () % 10 + 1;
    printf ("Generating %d random points\n", nPts);

    int nVoxels = raytracer.get_n_voxels ();
    Eigen::MatrixXf endpoints = Eigen::MatrixXf::Zero (nPts, 3);
    for (int i = 0; i < nPts; i ++)
    {
      // Generate a number bound by (0, 2 * size of number of voxels - 1), so
      //   there is 50% chance of picking a point in the point cloud.
      //int idx = rand () % (2 * nVoxels) + 1;
      int idx = rand () % nVoxels;

      // If generated index is out of bounds, this will be a random point.
      //if (idx >= nVoxels)
      //{
      //  endpoints.row (i) = Eigen::Vector3f::Random ();
      //}
      // Else, take this point in the voxels, then add some noise so it could
      //   fall outside of the point cloud
      //else
      //{
        pcl::PointXYZ pt;
        raytracer.get_voxel (idx, pt);
        Eigen::Vector3f noisy_pt = Eigen::Vector3f (pt.x, pt.y, pt.z);

        // Add noise
        std::cerr << "voxel: " << noisy_pt.transpose () << std::endl;
        // Random() returns float in range [-1, 1]
        Eigen::Vector3f noise = Eigen::Vector3f::Random () * noise_res;
        std::cerr << "noise: " << noise.transpose () << std::endl;
        endpoints.row (i) = noisy_pt + noise;
        std::cerr << "noisy_pt: " << endpoints.row (i) << std::endl;
      //}
    }

    // Do ray-trace occlusion test for each endpoint
    // Must test endpoints one by one, not an n x 3 matrix, `.` octree
    //   getIntersectedVoxelCenters() only takes one ray at a time.
    for (int i = 0; i < nPts; i ++)
    {
      std::cout << "Ray through " << endpoints.row (i) << std::endl;

      // Ray trace
      // Occluded = red arrow drawn in RViz, unoccluded = green
      bool occluded = raytracer.raytrace_occlusion_test (origin,
        endpoints.row (i));
      printf ("Occluded? %s\n", occluded ? "true" : "false");

      char enter;
      std::cout << "Press any character, then press enter: ";
      std::cin >> enter;
    }
  }

  return 0;
}

