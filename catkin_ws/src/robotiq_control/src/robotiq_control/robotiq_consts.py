#!/usr/bin/env python

# Mabel Zhang
# 2 May 2017
#
# Constants for Robotiq 3-finger hand
#


class RobotiqConsts:

  n_fingers = 3

  # Constants for input register gMOD
  #   http://support.robotiq.com/pages/viewpage.action?pageId=590045
  #   msg is in: robotiq_s_model_articulated_msgs SModelRobotInput
  MOD_BASIC = 0
  MOD_PINCH = 1
  MOD_WIDE = 2
  MOD_SCISSOR = 3

  # Actual finger position (input registers gPO_)
  POA_OPEN = 0
  POA_OPEN_THRESH = 6  # It never gets to strictly 0

  # Empirically found by echoing rostopic /SModelRobotInput gPOA gPOB gPOC when
  #   closing gripper.
  POA_CLOSE_BASIC = 241
  POA_CLOSE_PINCH = 113
  POA_CLOSE_WIDE = 240
  # Scissor mode, gMOD == 3. Then check gPOS for scissor joint
  POS_CLOSE = 232

  # Object detection (input registers gDT_), when Robotiq detects something
  #   when opening (0x01) or closing (0x02), it stops automatically no matter
  #   what you command the position to. So it's helpful to check this so you
  #   know why the hand isn't moving even though you commanded it to, e.g.
  #   during guarded_move!
  DT_MOVING = 0
  DT_STOPPED_OPEN = 1
  DT_STOPPED_CLOSE = 2
  DT_DONE = 3


  # tf frame names are finger_*_link_#, where * is <middle | 1 | 2>, # is <1 | 2 | 3>.

  # tf names middle = Robotiq finger A, finger_1 = Finger C, finger_2 = Finger B
  FINGER_FRAMES = ['middle', '2', '1']
  FINGER_FRAMES_SHORT = ['A', 'B', 'C']

  # Names on tf
  LINK_FRAME_FMT = 'finger_%s_link_%s'
  LINK_FRAMES = ['0', '1', '2', '3']

  # Names on rostopic /iiwa/joint_states
  JOINT_FRAME_FMT = 'finger_%s_joint_%s'  
  JOINT_FRAMES = ['1', '2', '3']

  # Short hand for my custom plotting
  LINK_FRAME_SHORT_FMT = 'f%sl%s'
  JOINT_FRAME_SHORT_FMT = 'f%sj%s'

  F1_PRESHAPE_FRAME = 'palm_finger_1_joint'
  F1_PRESHAPE_FRAME_SHORT = 'f1j0'
  F2_PRESHAPE_FRAME = 'palm_finger_2_joint'



