#!/usr/bin/env python

# Mabel Zhang
# 3 Feb 2019
#
# Transforms GraspIt RobotIQ local gripper frame to Gazebo Robotiq gripper
#   frame. This is a difference between the proprietary Inventor (.iv) CAD
#   files used by GraspIt's built-in Robotiq hand and the STL CAD files used
#   by Robotiq Gazebo simulation packages.
#

import numpy as np

# Note quaternions in this original version is ordered (w, x, y, z), unlike the
#   ROS version which is (x, y, z, w).
from util import tf_transformations


# Mapping found by moving each dof separately (by one-hot dofs vector) in
#   GraspIt.
# Gazebo sim has 12 joints, GraspIt has 11 dofs. The l_palm_finger_middle_joint
#   in Gazebo is unnecessary, thumb does not move sideways. GraspIt does not
#   have this dof. Can just always set that joint in Gazebo to position 0.
# Index: dof index in GraspIt RobotIQ returned Grasp.dofs
# Value: name of joint in Robotiq Gazebo simulation
robotiq_dofs_graspit_to_gz = ['l_finger_middle_joint_1',
  'l_finger_middle_joint_2',
  'l_finger_middle_joint_3',
  'l_palm_finger_1_joint',
  'l_finger_1_joint_1',
  'l_finger_1_joint_2',
  'l_finger_2_joint_3',
  'l_palm_finger_2_joint',
  'l_finger_2_joint_1',
  'l_finger_2_joint_2',
  'l_finger_1_joint_3']



#def graspit_robotiq_to_gz (mat_gi):

  # Guess -90 rot wrt x. Conventional approach direction is z, which I would
  #   guess GraspIt uses. Gazebo Robotiq model is weird that it has y pointing
  #   out of palm.
#  mat_trans = tf_transformations.euler_matrix (0, 0, 0, axes='rxyz')

#  mat_gz = np.dot (mat_gi, mat_trans)

#  return mat_gz

