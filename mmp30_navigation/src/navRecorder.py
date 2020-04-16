#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import csv
import rospkg 


class navRecorder:
    def __init__(self,file,createNew):
        
        #Subscriber to position
        self.navSub = rospy.Subscriber("/utm_odometry/odom", Odometry, self.navCallback)

        #CSV Handling
        self.file = str(file) + '.csv'
        self.createNew = createNew
        if self.createNew == True:
            with open(self.file,'w') as firstFile:
                firstFileWriter = csv.writer(firstFile)
                firstFileWriter.writerow(['X','Y'])
    
    def writeVal(self,x,y):
        with open(self.file,'a') as tempFile:
            tempFileWriter = csv.writer(tempFile)
            tempFileWriter.writerow([x,y])

    def navCallback(self,position):
        x = position.pose.pose.position.x
        y = position.pose.pose.position.y
        self.writeVal(x,y)
        print("wrote {} , {} to csv".format(x,y))






def recorder():
    rospy.init_node("recorder")
    rospack = rospkg.RosPack()
    navCSV_Path = rospack.get_path('mmp30_navigation') + '/data/navlog'
    navRecorder(navCSV_Path,True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown("shutdown initiated")



if __name__ == "__main__":
    recorder()