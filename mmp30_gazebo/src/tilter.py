#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from time import sleep


min = .3
max = .7

sleep_val = .005

def scanner():
    pub = rospy.Publisher('/nod_position_controller/command', Float64, queue_size=10)
    rospy.init_node('nodder', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        
        pos = min
        #go up to 1 m
    

        while (pos < max):
            pub.publish(pos)
            sleep(sleep_val)
            pos += .0005

        # #go down to -.5m 
        while (pos > min):
            pub.publish(pos)
            sleep(sleep_val)
            pos -= .0005
   

        

if __name__ == '__main__':
    try:
        scanner()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown()