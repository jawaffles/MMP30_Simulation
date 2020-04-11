#!/usr/bin/env python
import rospy
import rospkg
import pandas as pd

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

import time
from math import sqrt

class NavMaster:
    def __init__(self):
        "Initialize CSV waypoint Read"
        self.rospack = rospkg.RosPack()
        self.waypointCSV_Path = self.rospack.get_path('simple_navigation_goals') + '/nav_master_waypoints/waypoints.csv'
        self.waypoints = pd.read_csv(self.waypointCSV_Path)
        # print(self.waypoints)

        "Current waypoint that is being navigated towards"
        self.goal = 0

        "Initialize Goal Publisher"
        self.goalPub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)

        "Initialize Subscribers"
        self.posSub = rospy.Subscriber("/utm_odometry/odom", Odometry, self.positionCallback)

        self.statusSub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.statusCallback)    


        self.goal_status = 3 #1 = going 2 = waiting , 3 = reached
    
    def sendGoal(self,i):
        goal = PoseStamped()
        goal.header.frame_id = 'utm_relative'
        goal.header.stamp = rospy.Time.now()
        "Get appropriate waypoint values at row i (waypoint i)"
        goal.pose.position.x = self.waypoints.at[i,'X']
        goal.pose.position.y = self.waypoints.at[i,'Y']
        goal.pose.position.z = self.waypoints.at[i,'Z']
        goal.pose.orientation.x  = self.waypoints.at[i,'THETAX']
        goal.pose.orientation.y = self.waypoints.at[i,'THETAY']
        goal.pose.orientation.z = self.waypoints.at[i,'THETAZ']
        goal.pose.orientation.w = self.waypoints.at[i,'QUATW']
        self.goalPub.publish(goal)
        print(goal)

    def positionCallback(self,position):
        current_posex , current_posey = position.pose.pose.position.x , position.pose.pose.position.y
        goal_posex , goal_posey = self.waypoints.at[self.goal,'X'], self.waypoints.at[self.goal,'Y']

        dist2waypoint = sqrt((goal_posex - current_posex)**2+(goal_posey - current_posey)**2)

        # print(current_posex,current_posey)        
        # print(dist2waypoint)
        if dist2waypoint < 2 and self.goal_status == 3:
            self.sendGoal(self.goal)
            self.goal_status = 1
        elif self.goal_status == 1:
            print("going now")
            # pass
        else:
            print("still {} away".format(dist2waypoint))
            # pass

    def statusCallback(self,status):
        if len(status.status_list) != 0 : #checks if array is not empty
            if self.goal != 3 and status.status_list[0].status == 3:
                self.goal_status = 3
                self.goal += 1

        # nav_status = status.status_list
        # print(len(nav_status))
        # print(nav_status)



    


def main():

    "Initializes ROS Node"
    rospy.init_node('Navigation')

    "Instantiates NavMaster Class"
    nav = NavMaster()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        # print("")
        rospy.signal_shutdown("shutdown initiated")



if __name__ == '__main__':
    main()