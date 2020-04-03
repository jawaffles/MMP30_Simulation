#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from time import sleep


def scanner():
    pub = rospy.Publisher('/hokuyo_position_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        
        pos = -1.2
        #go up to 1 m
    

        while (pos < .1):
            pub.publish(pos)
            sleep(.01)
            pos += .0005

        # #go down to -.5m 
        while (pos > -1.2):
            pub.publish(pos)
            sleep(.01)
            pos -= .0005
   

        

if __name__ == '__main__':
    try:
        scanner()
    except rospy.ROSInterruptException:
        pass