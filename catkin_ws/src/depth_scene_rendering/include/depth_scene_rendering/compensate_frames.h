#ifndef _COMPENSATE_FRAMES_H_
#define _COMPENSATE_FRAMES_H_

// Mabel Zhang
// 11 Oct 2018
//
// Small transformations to compensate for different world frame orientations
//   in various software used in data generation and visualization, e.g.
//   Blender, GraspIt, so that they are all consistent and correct in result.
//


#include <pcl_ros/point_cloud.h>

// Custom
#include <util/pcd_util.h>  // flip_yz()


// Blender camera has computer graphics convention (x to the right, y up,
//   z pointing out of image. Image origin at lower-left), which is rotated
//   180 degrees wrt x-axis from robotics convention (x to the right, y down,
//   z poiniting outward from camera, into image. Image origin at upper-left).
void compensate_blensor_frame (pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud_p)
{
  flip_yz (cloud_ptr);
}

void compensate ()
{

}

void compensate_graspit_frame ()
{

}


#endif
