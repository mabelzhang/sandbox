#!/usr/bin/env python

# Mabel Zhang
# 18 Feb 2018
#
# Simulates TakkTile Robotiq sensor kit /calibrated rostopic
#
# Copied and modified from my reflex_gazebo reflex_driver_node.py
#

# ROS
import rospy
from std_srvs.srv import Empty, EmptyResponse

# Gazebo
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesRequest

# takktile_ros, driver for TakkTile Robotiq kit
from takktile_ros.msg import Touch

# My packages
from robotiq_s_model_articulated_gazebo_plugins.msg import Contact


class SimTakktileNode:

  def __init__ (self):#, hertz):

    # Published by contact_sensor_plugin.cpp
    rospy.Subscriber ('/left_hand/contact', Contact, self.contact_cb)

    # Published by takktile_ros takktile_node.py for real TakkTile sensor
    self.touch_pub = rospy.Publisher ('/takktile/calibrated', Touch,
      queue_size=5)

    #self.HERTZ_DESIRED = hertz

    self.N_SEN = 18

    # A constant, just to make it easier to see which sensors are fired.
    #   This should be above the threshold that you set for contact to be true.
    #   This is only set on sensors that have contact = True.
    self.PRESSED = 100.0

    # Time stamp when the current 1/HERTZ seconds segment started
    self.time_started = -1

    self.restart_timer = True

    self.contacts = []


  def reset_contacts (self):
    # Reset contacts array
    self.contacts = []
    for j in range (self.N_SEN):
      self.contacts.append (False)
    

  def contact_cb (self, msg):

    if not self.contacts:
      self.reset_contacts ()

    #print ('Got a Gazebo /left_hand/contact msg with contact on finger %d' % \
    #  (msg.fin_num))

    if self.restart_timer:

      self.restart_timer = False

      self.time_started = msg.header.stamp

      self.reset_contacts ()

    # Record this sensor was fired
    self.contacts [msg.sen_num] = True

    # TODO: Bug. This never updates... because it only updates fingers that get
    #   a contact. But for fingers that got a contact before, but no longer
    #   contacted now, the contact is not being resetted to False.


    #print ('Finger %d sensor %d fired' % (msg.fin_num, msg.sen_num))


  # reflex_driver_node.cpp calls it reflex_hand_state_cb(), but since this is
  #   not a callback function, the _cb suffix is removed.
  def publish_takktile (self):

    # If not initialized yet, that means no contacts yet. Just init all to 0s
    if not self.contacts:
      self.reset_contacts ()

    touch_msg = Touch ()


    #####
    # Tactile data
    #####

    for j in range (0, len (self.contacts)):

      # If got a contact from this sensor, set to true, else false.
      if self.contacts [j]:
        touch_msg.pressure.append (self.PRESSED)

        #print ('Sensor %d set to True' % j)

      else:
        touch_msg.pressure.append (0.0)

    # Debug: print first /takktile/calibrated msg
    #print (touch_msg)


    #####
    # Publish msg
    #####

    # Publish 10 times, `.` UDP drops packages
    for i in range (0, 10):
      self.touch_pub.publish (touch_msg)

    #print ('Published /reflex_hand msg')

    # Restart a time segment, after having published this one
    self.restart_timer = True


  # Not used
  '''
  # Calls Gazebo rosservice to get joint position
  # Returns (True, position[]) if call is successful, else (False, []).
  def get_joint_pos (self, joint_name):

    srv_name = '/gazebo/get_joint_properties'

    rospy.wait_for_service (srv_name)

    #rospy.loginfo ('Calling rosservice %s on %s...' % (srv_name, joint_name))
    try:
      srv = rospy.ServiceProxy (srv_name, GetJointProperties)
      req = GetJointPropertiesRequest ()
      req.joint_name = joint_name
      resp = srv (req)

    except rospy.ServiceException, e:
      rospy.logerr ('sample_gazebo remove_model(): Service call to %s failed: %s' %(srv_name, e))
      return (False, [])

    if resp.success:
      #rospy.loginfo ('rosservice call succeeded')
      return (True, resp.position)
    else:
      rospy.logerr ('rosservice call failed: %s' % resp.status_message)
      return (False, [])
  '''


  def zero_tactile_fn (self, req):

    rospy.loginfo ('Zeroing tactile data...')

    # Clear all contacts. This is needed, otherwise rostopics may not be
    #   quick enough to arrive contact_cb to clear array, then
    #   publish_takktile() would never publish a all-0 msg to clear
    #   the contacts! Then guarded_move in reflex_base.py might have outdated
    #   info from a prev contact, so it never closes the fingers with outdated
    #   contact=True info.
    self.reset_contacts ()

    # Publish a all-0-contacts msg to tell subscribers contacts are cleared
    self.publish_takktile ()

    return EmptyResponse ()



def main ():

  rospy.init_node ('sim_takktile_node', anonymous=True)

  #hertz = 30

  thisNode = SimTakktileNode ()

  zero_tactile_name = "/takktile/zero"
  rospy.loginfo ("Advertising the %s service", zero_tactile_name)
  zero_tactile = rospy.Service (zero_tactile_name, Empty,
    thisNode.zero_tactile_fn)


  wait_rate = rospy.Rate (30)
  while not rospy.is_shutdown ():

    thisNode.publish_takktile ()

    # If this doesn't get awaken, it's because your Gazebo physics is paused.
    #   Just unpause, and this will return from sleep.
    #   Ref: http://answers.ros.org/question/11761/rospysleep-doesnt-get-awaken/
    wait_rate.sleep ()


if __name__ == '__main__':

  # This enables Ctrl+C kill at any time
  try:
    main ()
  except rospy.exceptions.ROSInterruptException, err:
    pass

