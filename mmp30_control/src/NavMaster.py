#!/usr/bin/env python
import rospy
import rospkg
import pandas as pd

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

import time
from math import sqrt

#PCD Analysis imports
from PCD_CropRow import pcd_row_detection
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import ros_numpy

from matplotlib import pyplot as plt
import numpy as np
import open3d as o3d
from sklearn import linear_model, datasets

from datetime import datetime, timedelta



class NavMaster:
    def __init__(self):
        # "Initialize CSV waypoint Read"
        self.rospack = rospkg.RosPack()
        self.waypointCSV_Path = self.rospack.get_path('simple_navigation_goals') + '/nav_master_waypoints/waypoints.csv'
        self.waypoints = pd.read_csv(self.waypointCSV_Path)

        # "Current waypoint that is being navigated towards"
        self.goal = 0

        # Navigation Goal Publisher
        self.goalPub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)

        # Velocity Publisher
        self.velPub = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

        # Position Update Subscriber"
        self.posSub = rospy.Subscriber("/utm_odometry/odom", Odometry, self.positionCallback)

        # Auto Navigation Subscriber
        self.statusSub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.statusCallback)    

        # Actuated LiDAR Subscriber
        self.lidarSub = rospy.Subscriber("assembled_cloud2",PointCloud2,self.lidarCallback)
        
        # "State machine of Navigation
        self.goal_status = 2 #1 = AutoNaving 2 = liDAR Nav , 3 = AutoNav Reach
    
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

        if dist2waypoint < 1.5 and self.goal_status == 2:
            self.sendGoal(self.goal)
            self.goal_status = 1
        elif self.goal_status == 1:
            print("Auto Navigating Now still {} away".format(dist2waypoint))
            print("Current Goal is now {}".format(self.goal))
            time.sleep(5)
        elif self.goal_status == 2:
            print("Should be LiDAR naving still {} away".format(dist2waypoint))
            print("Current Goal is now {}".format(self.goal))

    def statusCallback(self,status):
        if len(status.status_list) != 0 : #checks if array is not empty
            if self.goal_status == 1 and status.status_list[0].status == 3:
                self.goal_status = 2
                self.goal += 1
                print("Current Goal is now {}".format(self.goal))
                if self.goal == len(self.waypoints.index):
                    print("ALL WAYPOINTS ACHIEVED")
                    rospy.signal_shutdown("Nav compelte shutdown")
        # nav_status = status.status_list
        # print(len(nav_status))
        # print(nav_status)

    def lidarCallback(self,point_cloud):
        if self.goal_status == 2:
            xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz_array)
            try:
                row_output = pcd_row_detection(pcd)
            except:
                row_output = [0,0,0]
                print("No point cloud detectioning, just going forward")
            heading_err = row_output[0]
            row_dist_err = row_output[1] - row_output[2]
            print("Heading Error: {}").format(row_output[0])
            print("Distance Error: {}").format(row_output[1] - row_output[2])
            print("Right row distance: {}").format(row_output[1])
            print("Left row distance: {}").format(row_output[2])
            self.navCorrection(heading_err,row_dist_err)
        else:
            pass 

    def navCorrection(self,heading_err,row_dist_err):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        vel_time = 2
        dist_corr = 0
        heading_corr = 0
        prop_head = 1.5
        prop_dist = .5

        if abs(row_dist_err) > .2:
            dist_corr = -row_dist_err * prop_dist
            print("Distance error off, correct vel send of {}").format(-row_dist_err*prop_dist)
        elif abs(heading_err) > .04:
            heading_corr = heading_err * prop_head
            print("heading error off, correct vel send of {}").format(heading_err*prop_head)
        else:
            print("Everything is fine going forward")
            dist_corr = 0
            heading_corr = 0
            vel_time =.25

        print("Complete Correction {}").format(dist_corr + heading_corr)

        vel_msg.angular.z = (dist_corr + heading_corr) 
        vel_msg.linear.x = .3
        start = datetime.now()
        while datetime.now() - start < timedelta(seconds=vel_time):
            self.velPub.publish(vel_msg)
    


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