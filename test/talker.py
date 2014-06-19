#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
from pandora_xmega_hardware_interface.msg import BatteryMsg

battery_topic = "sensors/battery"
sonars_topic = "sensors/range"

def talker():
    i = 4
    pub = rospy.Publisher('chatter',Int16 , queue_size=10)
    pub_batteries = rospy.Publisher(battery_topic, BatteryMsg ,queue_size=10)
    pub_sonars = rospy.Publisher(sonars_topic, Range,queue_size=10)
    rospy.init_node('talker', anonymous = False)
    r = rospy.Rate(0.5) # 2 hz
    
    batteries_msg = BatteryMsg()
    batteries_msg.voltage.append(float((random.randrange(19,39)))/4)
    batteries_msg.voltage.append((random.randrange(19,39))/4)
    
    sonars_msg_left = Range()
    sonars_msg_right = Range()
    sonars_msg_left.header.frame_id = "left"
    sonars_msg_right.header.frame_id = "right" 

    while not rospy.is_shutdown():
       
        rospy.loginfo("%d",i)

        batteries_msg.voltage[0]=(float((random.randrange(0,20))))/4 +19
        batteries_msg.voltage[1]=(float((random.randrange(0,20))))/4 +19
        
        sonars_msg_left.range = (float((random.randrange(0,20))))/4
        sonars_msg_right.range = (float((random.randrange(0,100))))/4
        
        pub.publish(i)
        pub_batteries.publish(batteries_msg)
        pub_sonars.publish(sonars_msg_left)
        r.sleep()
        pub_sonars.publish(sonars_msg_right)
        
        i = i+4
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
