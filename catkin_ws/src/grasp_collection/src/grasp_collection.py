#!/usr/bin/env python

# Mabel Zhang
# 26 Sep 2018
#
# Calls GraspIt to plan eigengrasps and save them to disk.
#

import os

import rospy
import rospkg

from graspit_commander import GraspitCommander

from graspit_interface.srv import LoadWorld

# Local
from config_consts import worlds


def main ():

  rospy.init_node ('graspit_commander')

  rospack = rospkg.RosPack ()
  pkg_path = rospack.get_path ('grasp_collection')


  #for i in range (len (worlds)):
  for i in range (1):

    # graspit_interface loadWorld automatically looks in worlds/ path under
    #   GraspIt installation path.
    # File name of world XML, one file per object
    #world_fname = os.path.join (pkg_path, 'graspit_input/worlds/dexnet/',
    #  worlds [i])
    world_fname = worlds [i]

    print ('Loading world from %s' % world_fname)

    # Load world
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
    # TODO: Make planGrasps() return more than just the top 20 grasps. Need
    #   low-quality ones as well.

    res = GraspitCommander.planGrasps ()
    print (type (res))
    print (res)




    # Get contact locations wrt object frame



    # Save grasps and contact locations to disk




if __name__ == '__main__':

  main ()

