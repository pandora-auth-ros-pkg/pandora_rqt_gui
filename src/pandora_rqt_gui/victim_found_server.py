#!/usr/bin/env python

import roslib
import rospy
import pandora_fsm
import actionlib
import fsm_communications.msg 
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot, QTime, QObject

class ValidateVictimAction(object):
  
  _result = fsm_communications.msg.ValidateVictimResult()
  _victimFound = False
  _operatorResponded = False
  _victimValid = False
  
  
  def __init__(self,name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name , fsm_communications.msg.ValidateVictimAction,
     execute_cb = self.execute_cb, auto_start = False)
    self._as.start()
    
  def execute_cb(self, goal):
    # store the Info
    self._victimInfo = [goal.victimFoundx , goal.victimFoundy , 4 , goal.probability , goal.sensorIDsFound]
    # helper variables
    success = True
      
      
    rospy.loginfo(" Executing the VictimValidation callback " )
    rospy.loginfo(self._victimInfo)
      

    if self._as.is_preempt_requested():
         rospy.loginfo('%s: Preempted' % self._action_name)
         self._as.set_preempted()
         success = False
           
    if  success:
      
      self._victimFound = True
      self.wait_for_response()
      self._victimFound = False
      self._operatorResponded = False
      self._result.victimValid = self._victimValid
      self._as.set_succeeded(self._result)
      
      
      
  def wait_for_response(self):
    while (not self._operatorResponded):
       i = 0
        
        
  def shutdown(self):
    pass
   #~ del(self._as)
   #~ del(self)
    
           
           
if __name__ == '__main__':
    rospy.init_node('victimValidation')
    ValidateVictimAction('victimValidation')
    rospy.spin()
           
      
      
      
      
    
    
    
    
