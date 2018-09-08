// Mabel Zhang
// 8 Sep 2018
//
// Inspect values of a pcd file
//

#include <iostream>     // std::cout
#include <cfloat>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Eigen (PCL underlying matrices)
#include <Eigen/Core>

// Custom
#include <util/io_util.h>  // join_paths ()
#include <util/pcd_util.h>  // load_cloud_file ()


int main (int argc, char ** argv)
{
  std::string cloud_path = "/media/master/Data_Ubuntu/courses/research/graspingRepo/train/visuotactile_grasping/2018-09-08-01-18-17_noisy00000.pcd";

  // Instantiate cloud
  pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_ptr =
    pcl::PointCloud <pcl::PointXYZ>::Ptr (
      new pcl::PointCloud <pcl::PointXYZ> ());

  // Load scene cloud
  load_cloud_file (cloud_path, cloud_ptr);
  printf ("Cloud size: %ld points\n", cloud_ptr->size ());
  printf ("Organized? %s\n", cloud_ptr->isOrganized () ? "true" : "false");

  // Inspect point cloud values
  float min_z = FLT_MAX, max_z = FLT_MIN;
  for (int i = 0; i < cloud_ptr -> size (); i ++)
  {
    if (cloud_ptr -> at (i).z < min_z)
      min_z = cloud_ptr -> at (i).z;
    if (cloud_ptr -> at (i).z > max_z)
      max_z = cloud_ptr -> at (i).z;
  }

  printf ("min z %f, max z %f\n", min_z, max_z);


  return 0;
}

