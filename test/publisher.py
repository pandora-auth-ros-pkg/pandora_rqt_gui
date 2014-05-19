#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16



class Publisher()


def __init__(self,isVictim):
  
  
  self.isVictim = bool
  self._pub = rospy.Publisher('chatter',Int16 , queue_size=10)
  rospy.init_node('VictimConfirm', anonymous=True)
  r = rospy.Rate(1) # 1 hz
  
  
  
  
def publish():
  
    
  while not rospy.is_shutdown():
       
        rospy.loginfo("VICTIM %s",)
        
        pub.publish(i)
        i = i+4
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
