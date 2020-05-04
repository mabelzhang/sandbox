#!/usr/bin/env python

# Mabel Zhang
# 20 May 2017
#
# Refactored from my robotiq_takktile guarded_move.py
#
# Rosservice and function for controlling robotiq. Complementary to the
#   keyboard interface provided by robotiq_s_model_control package.
#   This is programmatic API.
#
# Ref:
#   Robotiq output registers (command)
#     http://support.robotiq.com/pages/viewpage.action?pageId=590044
#   Robotiq input registers (status)
#     http://support.robotiq.com/pages/viewpage.action?pageId=590045


# ROS
import rospy
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse

from copy import deepcopy
import argparse

import numpy as np

# Copied from robotiq_s_model_control/nodes/SModelSimpleController.py
# Same definition as robotiq_s_model_articulated_msgs/SModelRobotOutput,
#   SModelRobotInput, but real robot listens to these.
# For real robot
from robotiq_s_model_control.msg import SModel_robot_output, SModel_robot_input
# For Gazebo sim robot
from robotiq_s_model_articulated_msgs.msg import SModelRobotOutput, SModelRobotInput

# My packages
from util.ansi_colors import ansi_colors

# Local
from robotiq_consts import RobotiqConsts as RC
from robotiq_control.srvs import SetPositions, SetPositionsResponse


# Call this before publishing an SModelRobotOutput, so we start at current
#   finger positions given by input registers and don't issue a position
#   that makes the fingers do a sudden jump.
# Update output registers to the current robot status as read from Robotiq
#   input registers.
def update_oregs (iregs, cmd, sim=False):

  if iregs is None:
    print ('ERROR: No Robotiq input registers status messages received on rostopic yet (toopic name depends on sim or real)')
    return False, cmd

  # Initialize command, so that all bits are 0.
  #   Very important to set unused bits to 0, according to Robotiq
  #   documentation.
  # If object already exists, don't reinitialize! Its rACT rMOD rGTO etc bits
  #   should remain the same. Otherwise you'll end up with a new object that
  #   has rACT=0, and deactivate the hand. Then in the next timestep it'll
  #   have to go through the activation routine again!
  if cmd is None:
    if sim:
      cmd = SModelRobotOutput ()
    else:
      cmd = SModel_robot_output ()

  # Copy the functional bits, activated, mode, and goto mode, from robot's
  #   current state, so we don't end up changing it!
  cmd.rACT = iregs.gACT
  cmd.rMOD = iregs.gMOD
  cmd.rGTO = iregs.gGTO

  # Set output position request (rPR) to finger's latest request (gPR)
  #   (Don't set it to finger's actual position (gPO), because the open
  #   position request is 0, while the actual never gets to 0, it stays at
  #   6.)
  cmd.rPRA = iregs.gPRA
  cmd.rPRB = iregs.gPRB
  cmd.rPRC = iregs.gPRC

  return True, cmd



# Do not instantiate this class directly. Use the rosservice calls.
class RobotiqControl:

  # Only used for set_pos sim-only service
  # Figures from robotiq_hand_description/urdf/robotiq_hand.urdf.xacro
  # [0, 70] degrees
  JOINT_1_MIN = 0
  JOINT_1_MAX = 1.2217
  JOINT_1_RANGE = JOINT_1_MAX - JOINT_1_MIN

  # [0, 90] degrees
  JOINT_2_MIN = 0
  JOINT_2_MAX = 1.5708
  JOINT_2_RANGE = JOINT_2_MAX - JOINT_2_MIN

  # [-30, 60] degrees
  JOINT_3_MIN = -0.6632
  JOINT_3_MAX = 1.0471
  JOINT_3_RANGE = JOINT_3_MAX - JOINT_3_MIN

  # [-17, 17] degrees
  PALM_F1_JOINT_MIN = -0.2967
  PALM_F1_JOINT_MAX = 0.2967
  PALM_F1_JOINT_RANGE = PALM_F1_JOINT_MAX - PALM_F1_JOINT_MIN

  # [-17, 17] degrees
  PALM_F2_JOINT_MIN = -0.2967
  PALM_F2_JOINT_MAX = 0.2967
  PALM_F2_JOINT_RANGE = PALM_F2_JOINT_MAX - PALM_F2_JOINT_MIN

  # Dummy joint. Not allowed to move
  PALM_FMIDDLE_JOINT_MIN = 0
  PALM_FMIDDLE_JOINT_MAX = 0


  def __init__ (self, sim=False):

    # Advertise rosservice
    self.activate_srv = rospy.Service ('robotiq_takktile/robotiq/activate',
      SetBool, self.activate_handler)
    self.deactivate_srv = rospy.Service ('robotiq_takktile/robotiq/deactivate',
      Trigger, self.deactivate_handler)

    self.reset_oregs_srv = rospy.Service ('robotiq_takktile/robotiq/reset_oregs',
      Trigger, self.reset_oregs_handler)

    self.open_srv = rospy.Service ('robotiq_takktile/robotiq/open', Trigger,
      self.open_handler)
    self.close_srv = rospy.Service ('robotiq_takktile/robotiq/close', Trigger,
      self.close_handler)

    self.basic_srv = rospy.Service ('robotiq_takktile/robotiq/basic', Trigger,
      self.basic_handler)
    self.pinch_srv = rospy.Service ('robotiq_takktile/robotiq/pinch', Trigger,
      self.pinch_handler)
    self.wide_srv = rospy.Service ('robotiq_takktile/robotiq/wide', Trigger,
      self.wide_handler)

    # Simulation-only, set joint positions to known grasp pose
    self.position_srv = rospy.Service ('robotiq_takktile/robotiq/set_pos',
      SetPositions, self.set_pos_handler)

    self.check_grasp_srv = rospy.Service (
      'robotiq_takktile/robotiq/check_grasp', Trigger, self.check_grasp_handler)


    self.sim = sim
    # Robotiq in Gazebo sim
    if self.sim:
      # Subscribe to Robotiq driver
      # Topic name defined in robotiq_s_model_articulated_gazebo_plugins/src/RobotiqHandPlugin.cpp
      #   Could be overwritten if .urdf / .xacro file specifies different
      #   <topic_state> in <plugin>.
      rospy.Subscriber ('/left_hand/state', SModelRobotInput, self.robotiq_cb)

      # Publish to Robotiq command
      # Topic name defined in robotiq_s_model_articulated_gazebo_plugins/src/RobotiqHandPlugin.cpp
      #   Could be overwritten if .urdf / .xacro file specifies different
      #   <topic_command> in <plugin>.
      self.robotiq_pub = rospy.Publisher ('/left_hand/command',
        SModelRobotOutput, queue_size=10)

    # Real Robotiq
    else:
      # Subscribe to Robotiq driver
      rospy.Subscriber ('SModelRobotInput', SModel_robot_input, self.robotiq_cb)

      # Publish to Robotiq command
      self.robotiq_pub = rospy.Publisher ('SModelRobotOutput',
        SModel_robot_output, queue_size=10)

    self.activated = False

    self.robotiq_iregs = None

    self.reset_fields ()


  def reset_fields (self):

    self.prev_enabled = None

    # Initialize command, so that all bits are 0.
    #   Very important to set unused bits to 0, according to Robotiq
    #   documentation.
    # Keep a member copy of the command, `.` next update will send the
    #   same command to hand, so the activate, preshape mode, etc bits are
    #   preserved as before!
    # Ref: Robotiq output registers (these are the ones you set,
    #   counter-intuitive)
    #   http://support.robotiq.com/pages/viewpage.action?pageId=590044
    if self.sim:
      self.robotiq_cmd = SModelRobotOutput ()
    else:
      self.robotiq_cmd = SModel_robot_output ()


  def get_prev_enabled (self):
    return self.prev_enabled


  # msg: Type robotiq_s_model_articulated_msgs/SModelRobotInput
  def robotiq_cb (self, msg):
    self.robotiq_iregs = deepcopy (msg)


  def get_iregs (self):
    return self.robotiq_iregs


  # Return latest published output commands, this way the caller does not miss
  #   any bits and reset them to 0, when a previous command set something to
  #   non-zero, for example!
  # NOTE caller should call this function any time they send a command via this
  #   class, to make sure they have the latest sent commands, so they don't
  #   overwrite anything! If you get weird behavior like fingers don't move,
  #   then check if you have called this function, if not, you should.
  def get_oregs (self):
    return self.robotiq_cmd


  def check_activated (self):
    if not self.activated:
      print ('ERROR: Hand not activated yet. Cannot open. Activate first.')
    return self.activated


  # req.data: whether to enable rICF bit, for individual control for fingers
  def activate_handler (self, req):
    success, err_msg = self.activate_hand (req.data)
    if success:
      self.activated = True
    return TriggerResponse (success, err_msg)


  # Parameters:
  #   rICF: Individual Control for Fingers, rICF output register on Robotiq.
  #     This allows each finger to be individually controlled.
  def activate_hand (self, rICF=0):

    while self.robotiq_iregs is None and not rospy.is_shutdown ():
      print ('activate_hand(): No Robotiq input registers status messages received on rostopic yet (toopic name depends on sim or real). Waiting...')
      try:
        rospy.sleep (0.1)
      except rospy.exceptions.ROSInterruptException, e:
        return False, 'Ctrl+C received'

    # If prev_enabled is not initialized yet (None),
    #   check if hand is already activated before we activate it. If already
    #   activated, then don't deactivate it at the end of this object's life!
    if self.prev_enabled is None:
      if self.robotiq_iregs.gACT == 1:
        self.prev_enabled = True
      else:
        self.prev_enabled = False

    # If hand is already activated, don't go through the calibration movements
    #   in activation (happens when publish rACT=1), just return
    if self.robotiq_iregs.gACT == 1:
      return True, 'Hand already activated'


    print ('Activating hand...')

    # Init robotiq_cmd to current hand state given by input registers
    success, self.robotiq_cmd = update_oregs (self.robotiq_iregs, self.robotiq_cmd)

    # Copied from robotiq_s_model_controller/nodes/SModelSimpleController.py
    # Note that as this is a new object, all of rPRA rPRB rPRC etc are 0s.
    #   Make sure that's what you want. You might want to set them to 
    #   self.robotiq_iregs.gPOA gPOB gPOC, if you want to restore the gripper
    #   to its original shape, after the calibration movements.
    # Ref output registers documentation:
    #   http://support.robotiq.com/pages/viewpage.action?pageId=590044
    self.robotiq_cmd.rACT = 1
    # "Go To" action moves fingers to requested position. Without this bit,
    #   you can only do activation, mode change, and automatic release.
    self.robotiq_cmd.rGTO = 1
    self.robotiq_cmd.rSPA = 255
    self.robotiq_cmd.rFRA = 150

    # Enable individual Control of Fingers. This allows each finger to be set
    #   at a different opening amount.
    # When rICF=0, all fingers close and open for the same amount, rPRA, as
    #   is done in the simplified controls in SModelSimpleController.py.
    self.robotiq_cmd.rICF = rICF

    # Publish command to Robotiq hand
    self.robotiq_pub.publish (self.robotiq_cmd)

    # Pause a few seconds for activation to finish
    if self.sim:
      sleep_time = 2
    else:
      sleep_time = 20
    print ('Waiting %d s for calibration movements to finish...' % sleep_time)
    rospy.sleep (sleep_time)
    print ('Resuming')

    return True, ''


  def deactivate_handler (self, req):
    success, err_msg = self.deactivate_hand ()
    if success:
      self.activated = False
    return TriggerResponse (success, err_msg)


  def deactivate_hand (self):

    while self.robotiq_iregs is None and not rospy.is_shutdown ():
      print ('deactivate_hand(): No Robotiq input registers status messages received on rostopic yet (toopic name depends on sim or real). Waiting...')
      try:
        rospy.sleep (0.1)
      except rospy.exceptions.ROSInterruptException, e:
        return False, 'Ctrl+C received'

    print ('Deactivating hand...')

    # Init robotiq_cmd to current hand state given by input registers
    # Instead of re-initializing a SModel_robot_output() object, which sets all
    #   bits to 0, it's safer to update_oregs() instead. `.` sometimes might
    #  just want to stop Robotiq motion, and that's the reason for deactivating.
    success, self.robotiq_cmd = update_oregs (self.robotiq_iregs, self.robotiq_cmd)

    # Copied from robotiq_s_model_controller/nodes/SModelSimpleController.py
    self.robotiq_cmd.rACT = 0

    self.robotiq_pub.publish (self.robotiq_cmd)

    return True, ''


  def reset_oregs_handler (self, req):
    success, err_msg = self.reset_oregs ()
    return TriggerResponse (success, err_msg)


  # Purpose: Sometimes when a wrong output registers command is issued,
  #   e.g. from guarded_move.py, then it calls update_oregs() to output
  #   subsequent commands based on current state in input registers, so if
  #   the hand is in a wrong state, e.g. fingers ABC are not in the same PR_
  #   even if they are supposed to. Then the output registers continue to
  #   issue the wrong positions, and hand never recovers.
  # This function lets user issue a rosservice call on the command line to
  #   re-zero output register commands. These are just commands, so the
  #   current state of hand doesn't matter. This gives user flexibility to
  #   decide when to zero, instead of zeroing at the beginning of each guarded
  #   move, as sometimes you might not want to reopen all fingers before a
  #   guarded move.
  def reset_oregs (self):

    if self.sim:
      self.robotiq_cmd = SModelRobotOutput ()
    else:
      self.robotiq_cmd = SModel_robot_output ()

    # Copy the functional bits, activated, mode, and goto mode, from robot's
    #   current state, so we don't end up changing it!
    self.robotiq_cmd.rACT = self.robotiq_iregs.gACT
    self.robotiq_cmd.rMOD = self.robotiq_iregs.gMOD
    self.robotiq_cmd.rGTO = self.robotiq_iregs.gGTO

    # Keep position requests (PR_) at 0

    self.robotiq_pub.publish (self.robotiq_cmd)

    return True, ''


  def open_handler (self, req):

    self.check_activated ()
    success, err_msg = self.open ()

    return TriggerResponse (success, err_msg)


  def open (self):

    if self.robotiq_cmd is None:
      err_msg = 'ERROR: robotiq_cmd object not initialized yet. Cannot get rICF bit, needed to open fingers.'
      print (err_msg)
      return False, err_msg

    print ('Opening gripper...')

    # Init robotiq_cmd to current hand state given by input registers
    success, self.robotiq_cmd = update_oregs (self.robotiq_iregs, self.robotiq_cmd)
    if not success:
      err_msg = 'ERROR: No Robotiq input registers status messages received on rostopic yet (toopic name depends on sim or real). Waiting...'
      print (err_msg)
      return False, err_msg

    # If fingers not individually controlled, just need to set rPRA to open all
    if self.robotiq_cmd.rICF == 0:

      # Move with default force and speed
      self.robotiq_cmd.rSPA = 255
      self.robotiq_cmd.rFRA = 150
     
      # Copied from robotiq_s_model_control/nodes/SModelSimpleController.py.
      self.robotiq_cmd.rPRA = 0

    # If fingers individually controlled, set all three fingers to 0 (open)
    else:
      # Move with default force and speed
      self.robotiq_cmd.rSPA = 255
      self.robotiq_cmd.rFRA = 150
      self.robotiq_cmd.rSPB = 255
      self.robotiq_cmd.rFRB = 150
      self.robotiq_cmd.rSPC = 255
      self.robotiq_cmd.rFRC = 150

      self.robotiq_cmd.rPRA = 0
      self.robotiq_cmd.rPRB = 0
      self.robotiq_cmd.rPRC = 0

    self.robotiq_pub.publish (self.robotiq_cmd)
    rospy.sleep (5)

    return True, ''


  def close_handler (self, req):

    self.check_activated ()
    success, err_msg = self.close ()

    return TriggerResponse (success, err_msg)


  def close (self):

    if self.robotiq_cmd is None:
      err_msg = 'ERROR: robotiq_cmd object not initialized yet. Cannot get rICF bit, needed to close fingers.'
      print (err_msg)
      return False, err_msg

    print ('Closing gripper...')

    # Init robotiq_cmd to current hand state given by input registers
    success, self.robotiq_cmd = update_oregs (self.robotiq_iregs, self.robotiq_cmd)

    # If fingers not individually controlled, just need to set rPRA to open all
    if self.robotiq_cmd.rICF == 0:

      # Move with default force and speed
      self.robotiq_cmd.rSPA = 255
      self.robotiq_cmd.rFRA = 150
     
      # Copied from robotiq_s_model_control/nodes/SModelSimpleController.py.
      self.robotiq_cmd.rPRA = 255

    # If fingers individually controlled, set all three fingers to 255 (closed)
    else:
      # Move with default force and speed
      self.robotiq_cmd.rSPA = 255
      self.robotiq_cmd.rFRA = 150
      self.robotiq_cmd.rSPB = 255
      self.robotiq_cmd.rFRB = 150
      self.robotiq_cmd.rSPC = 255
      self.robotiq_cmd.rFRC = 150

      self.robotiq_cmd.rPRA = 255
      self.robotiq_cmd.rPRB = 255
      self.robotiq_cmd.rPRC = 255

    self.robotiq_pub.publish (self.robotiq_cmd)
    rospy.sleep (5)

    return True, ''


  def pinch_handler (self, req):

    self.check_activated ()
    success, err_msg = self.preshape (RC.MOD_PINCH)

    return TriggerResponse (success, err_msg)


  def basic_handler (self, req):

    self.check_activated ()
    success, err_msg = self.preshape (RC.MOD_BASIC)

    return TriggerResponse (success, err_msg)


  def wide_handler (self, req):

    self.check_activated ()
    success, err_msg = self.preshape (RC.MOD_WIDE)

    return TriggerResponse (success, err_msg)


  # Put gripper in pinch mode, by setting output register rMOD
  # Parameters:
  #   rMOD: value to set output register rMOD to. MOD_PINCH, MOD_BASIC, or
  #     MOD_WIDE.
  def preshape (self, rMOD):

    # First open gripper, if not opened yet
    if self.robotiq_iregs.gPOA > RC.POA_OPEN_THRESH:
      print ('Fingers not opened yet. Opening them...')
      success, err_msg = self.open ()
      if not success:
        return success, err_msg


    print ('Moving forefingers preshape...')

    # Init robotiq_cmd to current hand state given by input registers
    success, self.robotiq_cmd = update_oregs (self.robotiq_iregs, self.robotiq_cmd)
    if not success:
      err_msg = 'ERROR: No Robotiq input registers status messages received on rostopic yet (toopic name depends on sim or real)'
      print (err_msg)
      return False, err_msg

    # Move with default force and speed
    self.robotiq_cmd.rSPA = 255
    self.robotiq_cmd.rFRA = 150

    self.robotiq_cmd.rMOD = rMOD

    self.robotiq_pub.publish (self.robotiq_cmd)
    rospy.sleep (3)

    return True, ''


  def set_rICF (self, rICF):

    print ('Setting rICF bit...')

    # Init robotiq_cmd to current hand state given by input registers
    success, self.robotiq_cmd = update_oregs (self.robotiq_iregs,
      self.robotiq_cmd)

    self.robotiq_cmd.rICF = rICF

    #print (self.robotiq_cmd)

    # Publish command to Robotiq hand
    self.robotiq_pub.publish (self.robotiq_cmd)
    rospy.sleep (2)


  # Simulation-only. Try to set joints to specified positions for each DOF.
  #   Caveat: Robotiq is compliant, distal link cannot be commanded directly.
  # Parameters:
  #   req: robotiq_control/SetPositions rossrv type
  def set_pos_handler (self, req):

    if not self.sim:
      success = False
      err_msg = 'ERROR: set_positions service is for simulation only.'

    if self.robotiq_cmd.rICF == 0:
      success = False
      err_msg = 'ERROR: Cannot set per-DOF positions when rICF bit is 0. Reactivate with rICF=1, then retry.'

    else:
      success, err_msg = self.set_pos (req.positions)

    return SetPositionsResponse (success, err_msg)


  # Caveat: Robotiq is compliant, distal link cannot be commanded directly.
  # Parameters:
  #   positions: std_msgs/Float32[], size 11 for 11 DOFs
  def set_pos (self, positions):

    success = 
    err_msg = ''

    if not self.sim:
      success = False
      err_msg = 'ERROR: set_positions service is for simulation only.'
      return success, err_msg

    # Move with default force and speed
    self.robotiq_cmd.rSPA = 255
    self.robotiq_cmd.rFRA = 150
    self.robotiq_cmd.rSPB = 255
    self.robotiq_cmd.rFRB = 150
    self.robotiq_cmd.rSPC = 255
    self.robotiq_cmd.rFRC = 150


    # Calculate preshape (palm) joints integer command
     
    # NOTE: Gazebo plugin says ICS indivudal scissor mode is not supported.
    #   So this only works if GraspIt only ever gives mirrored preshape joints
    #   for fingers 1 and 2.
    f1_preshape = int (((positions.palm_finger_1_joint - \
      self.PALM_F1_JOINT_MIN) / self.PALM_F1_JOINT_RANGE) * 255)
    f2_preshape = int (((positions.palm_finger_2_joint - \
      self.PALM_F2_JOINT_MIN) /  self.PALM_F2_JOINT_RANGE) * 255)
   
    # ICS not supported. Two forefingers must have mirrored values, f1 == -f2
    if (abs (f1_preshape + f2_preshape) > 1e-2):
      success = False
      err_msg = 'ERROR: Individual scissor mode not supported in simulation. Cannot set finger 1 and 2 to different palm prehape values.'
      return success, err_msg

    # Set command bits in msg
    self.robotiq_cmd.rMOD = rMOD


    # Calculate finger joints integer command

    # Command positions of proximal links (joint 1). Joint 2 and 3 are compliant
    # TODO: Forgot which one of 123 is ABC, see 2017 notes
    #   rPRA, rPRB, rPRC
 
    self.robotiq_cmd.rPR = int (((positions.finger_middle_joint_1 - \
      self.JOINT_1_MIN) / self.JOINT_1_RANGE) * 255)
    #self.robotiq_cmd.rPR = int (((positions.finger_middle_joint_2 - \
    #  self.JOINT_2_MIN) / self.JOINT_2_RANGE) * 255)
    #self.robotiq_cmd.rPR = int (((positions.finger_middle_joint_3 - \
    #  self.JOINT_3_MIN) / self.JOINT_3_RANGE) * 255)

    self.robotiq_cmd.rPR = int (((positions.finger_1_joint_1 - \
      self.JOINT_1_MIN) / self.JOINT_1_RANGE) * 255)
    #self.robotiq_cmd.rPR = int (((positions.finger_1_joint_2 - \
    #  self.JOINT_2_MIN) / self.JOINT_2_RANGE) * 255)
    #self.robotiq_cmd.rPR = int (((positions.finger_1_joint_3 - \
    #  self.JOINT_3_MIN) / self.JOINT_3_RANGE) * 255)
   
    self.robotiq_cmd.rPR = int (((positions.finger_2_joint_1 - \
      self.JOINT_1_MIN) / self.JOINT_1_RANGE) * 255)
    #self.robotiq_cmd.rPR = int (((positions.finger_2_joint_2 - \
    #  self.JOINT_2_MIN) / self.JOINT_2_RANGE) * 255)
    #self.robotiq_cmd.rPR = int (((positions.finger_2_joint_3 - \
    #  self.JOINT_3_MIN) / self.JOINT_3_RANGE) * 255)


 
    # Continue commanding until the fully actuated joints are within tolerance
    #   threshold of commanded joint positions
    # TODO: Make another function that subscribes to joint_states from Gazebo
    #   plugin, activated only when self.sim==True, and sets member variables.
    #   Here, compare to the member variables.
    # TODO: Might need a termination condition or timeout, in case something
    #   gets stuck.
    while (not self.actual_matches_commanded (positions)):
    
      # Set finger joints
      self.robotiq_pub.publish (self.robotiq_cmd)
      rospy.sleep (0.2)  # TODO: tune


    return success, err_msg


  def actual_matches_commanded (self, commanded):

    matched = False

    # Tolerance of how much off actual position is from commanded position.
    #   Distal links are expected to be off, since they are compliant, cannot
    #   command them.
    tolerance = 0.1  # TODO: Tune


    # TODO: self.actual needs to be
    #   set by a callback function that subscribes to Gazebo /joint_states.

    # Only joint_1 is meaningful to check. joint 2 and 3 are compliant and
    #   cannot be commanded.
    if (abs (commanded.finger_middle_joint_1 - actual...) < tolerance) and \
       (abs (commanded.finger_1_joint_1 - actual...) < tolerance) and \
       (abs (commanded.finger_2_joint_1 - actual...) < tolerance):
      matched = True
    else:
      # Print the differences of how far they're off

    # Just print these
    commanded.finger_middle_joint_2 - actual...
    commanded.finger_1_joint_2 - actual...
    commanded.finger_2_joint_2 - actual...

    commanded.finger_middle_joint_3 - actual...
    commanded.finger_1_joint_3 - actual...
    commanded.finger_2_joint_3 - actual...

    return matched


  # Check if a grasp is successful
  # Very simple logic. If fingers are all the way closed, then grasp failed -
  #   nothing was grasped between the fingers. Else, check the DT_ bit of
  #   Robotiq, which is for object detection. If an object is detected,
  #   grasp succeeded. Although, Robotiq manual says DT_ bit might not work
  #   for certain cases (forgot what, maybe small objects or ones in palm or
  #   something). So safest way is to check if all fingers are closed, and
  #   no objects are detected, then graps failed. Else success.
  def check_grasp_success (self):

    # Check gPOA for normal modes, gPOS for scissor mode, for closing fingers
    if self.robotiq_iregs.gMOD == RC.MOD_PINCH:
      POA_closed = RC.POA_CLOSE_PINCH
    elif self.robotiq_iregs.gMOD == RC.MOD_BASIC:
      POA_closed = RC.POA_CLOSE_BASIC
    elif self.robotiq_iregs.gMOD == RC.MOD_WIDE:
      POA_closed = RC.POA_CLOSE_WIDE
    elif self.robotiq_iregs.gMOD == RC.MOD_SCISSOR:
      POA_CLOSED = RC.POS_CLOSE

    #TODO: This check is useless... every grasp is successful based on this...............

    # If >= 2 fingers are above closed threshold, gripper is closed
    n_fingers_closed = np.count_nonzero (np.array ([self.robotiq_iregs.gPOA,
      self.robotiq_iregs.gPOB, self.robotiq_iregs.gPOC]) > POA_closed)
    print ('Robotiq registers:')
    print ([self.robotiq_iregs.gPOA, self.robotiq_iregs.gPOB, self.robotiq_iregs.gPOC])
    print ('Robotiq: number of fingers closed (registers > %g)? %d' % (
      POA_closed, n_fingers_closed))

    if n_fingers_closed >= 2:
      closed = True
    else:
      closed = False

    # The object detection bit is USELESS. Don't use it!!!
    # If any gDT_ bit is True, an object is detected
    #obj_detected = np.any (np.array ([self.robotiq_iregs.gDTA,
    #  self.robotiq_iregs.gDTB, self.robotiq_iregs.gDTC]))
    #print ('Robotiq: object detected in hand')

    #if closed and not obj_detected:
    if closed:
      #print ('%sGrasp unsuccessful%s' % (ansi_colors.FAIL, ansi_colors.ENDC))
      print ('%sGrasp unsuccessful%s' % (ansi_colors.LOW_RED, ansi_colors.ENDC))
      return False
    else:
      #print ('%sGrasp successful%s' % (ansi_colors.LOW_GREEN, ansi_colors.ENDC))
      print ('%sGrasp successful%s' % (ansi_colors.OKCYAN, ansi_colors.ENDC))
      return True


  def check_grasp_handler (self, req):

    grasp_success = self.check_grasp_success ()

    return TriggerResponse (grasp_success, '')



def main ():

  rospy.init_node ('robotiq_control', anonymous=True)


  arg_parser = argparse.ArgumentParser ()

  arg_parser.add_argument ('--sim', action='store_true', default=False, 
    help='Specify this to control Robotiq gripper in Gazebo simulation')

  # To work with roslaunch
  args = arg_parser.parse_args (rospy.myargv () [1:])

  if args.sim:
    print ('sim=True. Will use rostopics for Gazebo sim')


  this_node = RobotiqControl (args.sim)

  print ('%s initialized. Listening to service calls in namespace %s...' % (
    rospy.get_name (), rospy.get_namespace ()))

  while not rospy.is_shutdown ():
    try:
      rospy.sleep (0.1)
    except rospy.exceptions.ROSInterruptException, e:
      break


  # If hand was previously disabled before this script started, deactivate hand
  #   when quit
  if this_node.get_prev_enabled () is False:
    this_node.deactivate_hand ()


if __name__ == '__main__':
  main ()

