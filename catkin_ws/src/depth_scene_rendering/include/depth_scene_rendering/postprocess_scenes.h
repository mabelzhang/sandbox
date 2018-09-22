#ifndef _POSTPROCESS_SCENES_H_
#define _POSTPROCESS_SCENES_H_

// Mabel Zhang
// 22 Sep 2018
//
//
//

// Load camera matrix wrt object center
// extrinsics_path: path to .txt file containing matrix of camera pose wrt
//   object
// P: camera projection matrix
// p_obj_2d: return value. 2D image coordinates of object in image.
void calc_object_pose_wrt_cam (const std::string scene_path,
  Eigen::MatrixXf & P, Eigen::VectorXf & p_obj_2d, int rows, int cols)
{
  // Path to camera transformation matrix wrt object.
  std::string extrinsics_path;
  replace_ext (scene_path, ".txt", extrinsics_path);

  // 4 x 4
  Eigen::MatrixXf T_o_c;
  load_nx4_matrix (extrinsics_path, 4, T_o_c);
  //std::cerr << "T_o_c:\n" << T_o_c << std::endl << std::endl;

  // With this, pixel is flipped wrt y axis. Without this, pixel is flipped
  //   wrt x axis. Instead, just flip both x and y at end of fn, that works.
  /*
  // Invert to compensate for the pi rotation wrt y-axis in
  //   scene_generation.py save_extrinsics_from_pose
  Eigen::Matrix4f R_flipY;
  R_flipY << cos(M_PI), 0, sin(M_PI), 0,
             0, 1, 0, 0,
             -sin(M_PI), 0, cos(M_PI), 0,
             0, 0, 0, 1;
  T_o_c *= R_flipY;
  */

  // Invert T^o_c to get object 3D pose wrt camera, T^c_o
  Eigen::MatrixXf T_c_o = T_o_c.inverse ();
  //std::cerr << "T_c_o:\n" << T_c_o << std::endl << std::endl;

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

  p_obj_2d [0] = cols - p_obj_2d [0];
  p_obj_2d [1] = rows - p_obj_2d [1];
}

#endif
