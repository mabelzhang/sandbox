#!/usr/bin/env python

# Mabel Zhang
# 3 Feb 2018
#
# Move a gripper in GraspIt to one-hot dof vector, to help figure out which
#   dof is which joint.
#


import numpy as np

# GraspIt
from graspit_commander_custom.graspit_commander_custom import GraspitCommander

# Local
from grasp_collection.grasp_io import GraspIO


def main ():

  GraspitCommander.clearWorld ()
  GraspitCommander.importRobot ('RobotIQ')

  robot = GraspitCommander.getRobot ().robot
  n_dofs = len (robot.dofs)

  for i in range (n_dofs):

    print ('dof %d' % i)
 
    one_hot = np.zeros ((n_dofs, ))
    # Just assume some large number obvious enough to see. Hopefully GraspIt
    #   stops it before the joint goes beyond upper limit.
    one_hot [i] = 1

    # Must set dofs to loaded ones, otherwise fingers won't close half of
    #   the time, and won't reproduce the recorded contacts!!
    #GraspitCommander.forceRobotDof (grasps [g_i].dofs)
 
    # This gets a lot more contacts!!! the Warning below prints every
    #   time, always more than the number saved!!!
    GraspitCommander.moveDOFToContacts (one_hot, [10] * len (one_hot), False)
  

    raw_input ('Press enter to continue: ')


if __name__ == '__main__':
  main ()
