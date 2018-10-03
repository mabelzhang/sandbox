#!/usr/bin/env python

# Mabel Zhang
# 26 Sep 2018
#
# Calls GraspIt to plan eigengrasps and save them to disk.
#

import os

import rospy
import rospkg
from geometry_msgs.msg import Pose

from graspit_commander_custom.graspit_commander_custom import GraspitCommander

from graspit_interface.srv import LoadWorld

# Local
from config_consts import worlds


def main ():

  rospy.init_node ('graspit_commander')

  rospack = rospkg.RosPack ()
  pkg_path = rospack.get_path ('grasp_collection')


  #for i in range (len (worlds)):
  for i in range (1, 2):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    #world_fname = os.path.join (pkg_path, 'graspit_input/worlds/dexnet/',
    #  worlds [i])
    world_fname = worlds [i]

    print ('Loading world from %s' % world_fname)

    # Load world
    GraspitCommander.clearWorld ()
    GraspitCommander.loadWorld (world_fname)


    # Pros: This returns result directly to me. GraspitCommands returns nothing
    '''
    rospy.wait_for_service ('/graspit/loadWorld')
    try:
      load_world_srv = rospy.ServiceProxy ('/graspit/loadWorld', LoadWorld)
      resp = load_world_srv (world_fname)
      if result.result != LoadWorld._response_class.RESULT_SUCCESS:
        print ('ERROR: Failed to load world file %s' % world_fname)

    except rospy.ServiceException, e:
      print ('ERROR: Exception while calling /graspit/loadWorld service.')
    '''


    # Plan Eigengrasps

    # TODO Try different quality measures.

    # Returns graspit_interface_custom/action/PlanGrasps.action result.
    # Request for more than the default top 20 grasps, to get low-quality ones
    #   as well.
    res = GraspitCommander.planGrasps (n_best_grasps=50)
    print (type (res))
    print ('Returned %d grasps. First one:' % (len (res.grasps)))
    print (res.grasps [0])
    print (res.energies)
    print (res.search_energy)


    # To view each grasp in GraspIt GUI, for debugging, move hand to the pose
    #print ('Showing grasps in GraspIt GUI, one by one')
    #for i in range (len (res.grasps)):
    #print ('Grasp %d: energy %g' % (i, res.energies [i]))
    #  setRobotPose (res.grasps [i])
    #  uinput = raw_input ('Press enter to view next grasp, q to stop viewing')
    #  if uinput.lower () == 'q':
    #    break



    # Get contact locations wrt object frame
    # /graspit/findInitialContact ?
    # There is /graspit/moveDOFToContacts, but where are the contacts? They
    #   seem to be at the blue lines in the GUI, but are they exposed?



    # Save grasps and contact locations to disk




if __name__ == '__main__':

  main ()

