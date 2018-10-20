#ifndef _POSTPROCESS_SCENES_H_
#define _POSTPROCESS_SCENES_H_

// Mabel Zhang
// 22 Sep 2018
//
// Transform object pose into camera frame, using camera extrinsics matrix
//

// Local
#include <depth_scene_rendering/camera_info.h>


// Inverse matrix of extrinsics (extrinsics is saved wrt object frame).
void calc_object_pose_wrt_cam (const std::string scene_path,
  Eigen::MatrixXf & P, Eigen::MatrixXf & T_c_o, int rows, int cols)
  //bool flip=false)
{
  // Path to camera transformation matrix wrt object.
  // Extrinsics matrix is saved to same name as .pcd, but with .txt extension
  std::string extrinsics_path;
  replace_ext (scene_path, ".txt", extrinsics_path);

  // 4 x 4
  Eigen::MatrixXf T_o_c;
  load_extrinsics (extrinsics_path, 4, T_o_c);
  //std::cerr << "T_o_c:\n" << T_o_c << std::endl << std::endl;

  // With this, pixel is flipped wrt y axis. Without this, pixel is flipped
  //   wrt x axis. Instead, just flip both x and y at end of fn. That works.
  /*
  // Invert to compensate for the pi rotation wrt y-axis in
  //   scene_generation.py save_extrinsics_from_pose(), to make identity
  //   camera pose actually identity in extrinsics, wasn't sure why needed.
  // scene_generation.py writes to file:
  //   T_o_cam = T_W_obj^-1 * (T_W_cam * R_flipY)
  //   To recover T_W_cam, camera pose wrt world (or object, equivalent in my
  //     setup which places object at world origin):
  //   T_W_obj * T_o_cam * R_flipY^-1 = T_W_cam
  Eigen::Matrix4f R_flipY;
  R_flipY << cos(M_PI), 0, sin(M_PI), 0,
             0, 1, 0, 0,
             -sin(M_PI), 0, cos(M_PI), 0,
             0, 0, 0, 1;
  T_o_c *= R_flipY;
  */

  // Invert T^o_c to get object 3D pose wrt camera, T^c_o
  T_c_o = T_o_c.inverse ();
  //std::cerr << "T_c_o:\n" << T_c_o << std::endl << std::endl;

  /*
  // Flip in camera frame in 3D here, instead of flipping in 2D image plane
  //   project_3d_pose_to_2d().
  if (flip)
  {
    T_c_o (0, 3) = -T_c_o (0, 3);
    T_c_o (1, 3) = -T_c_o (1, 3);
  }
  */
}

// Project a 3D point to 2D
// T_c_o: 4 x 4 3D transform of object in camera frame
// P: 3 x 4 camera projection matrix (intrinsics)
// p_obj_2d: return value. 2D image coordinates of object in image.
void project_3d_pose_to_2d (Eigen::MatrixXf T_c_o,
  Eigen::MatrixXf & P, Eigen::VectorXf & p_obj_2d, int rows, int cols,
  bool flip=false)
{
  // Last column is object 3D position wrt camera
  Eigen::VectorXf p_c_obj = T_c_o.col (3);
  //std::cerr << "obj_pos:\n" << p_c_obj << std::endl << std::endl;

  // Project 3D position to 2D image pixel using projection matrix
  // 3 x 1
  Eigen::VectorXf p_obj = P * p_c_obj;
  //std::cerr << "P * obj_pos:\n" << p_obj << std::endl << std::endl;
  // (u/w, v/w, 1, 1)
  p_obj /= (p_obj [2]);
  //std::cerr << "obj_pos:\n" << p_obj << std::endl << std::endl;
  // (u/w, v/w)
  p_obj_2d = p_obj.topRows (2);
  //std::cerr << "obj_pos_2d:\n" << p_obj_2d << std::endl << std::endl;

  // Need to flip x and y, `.` Blender camera faces -z, even though did
  //   flip_yz() after loading point cloud, intrinsics matrix is based on orig
  //   point cloud, so need to flip z manually, if I remember correctly.
  if (flip)
  {
    p_obj_2d [0] = cols - p_obj_2d [0];
    p_obj_2d [1] = rows - p_obj_2d [1];
  }
}

// Load camera matrix wrt object center
// extrinsics_path: path to .txt file containing matrix of camera pose wrt
//   object
// P: camera projection matrix
// p_obj_2d: return value. 2D image coordinates of object in image.
void calc_object_pose_in_img (const std::string scene_path,
  Eigen::MatrixXf & P, Eigen::VectorXf & p_obj_2d, int rows, int cols,
  bool flip=false)
{
  // Transform point from object frame to camera frame, using extrinsics matrix
  Eigen::MatrixXf T_c_o;
  calc_object_pose_wrt_cam (scene_path, P, T_c_o, rows, cols); //, flip);

  project_3d_pose_to_2d (T_c_o, P, p_obj_2d, rows, cols, flip);
}

#endif
