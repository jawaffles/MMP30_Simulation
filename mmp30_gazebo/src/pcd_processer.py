#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from time import sleep
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist
import ros_numpy

# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

# import matplotlib.pyplot as plt
from matplotlib import pyplot as plt
import numpy as np
import time
import open3d as o3d
from sklearn import linear_model, datasets
from test import pcd_row_detection

from datetime import datetime, timedelta

from mmp30_gazebo.srv import row_angle

# import ros_numpy

global pub


def vel_corr(heading_err,row_dist_err):
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()


    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    vel_time = 1.5
    dist_corr = 0
    heading_corr = 0
    prop_head = 1.5
    prop_dist = .5
    # print("heading error {}").format(heading_err)

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
        # vel_msg.linear.x = .3

    print("Complete Correction {}").format(dist_corr + heading_corr)

    vel_msg.angular.z = (dist_corr + heading_corr) 
    vel_msg.linear.x = .3
    start = datetime.now()
    while datetime.now() - start < timedelta(seconds=vel_time):
        velocity_publisher.publish(vel_msg)   




def callback(point_cloud):
    # rospy.wait_for_service('row_angle')
    #     try:
    #         ra_nodehandle = rospy.ServiceProxy('row_angle', row_angle)
    #         resp = ra_nodehandle(point_cloud)

    #         print("RETURNED ANGLE: {} ").format(resp)

    #     except rospy.ServiceException, e:
    #         print "Servce call failed: %s"%e

    # print(point_cloud.)    

    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud)
    
    print(len(xyz_array))

    pcd = o3d.geometry.PointCloud()
    
    pcd.points = o3d.utility.Vector3dVector(xyz_array)

    row_output = pcd_row_detection(pcd)

    # heading_err = row_output[0]
    # row_dist_err = row_output[1] - row_output[2]
    # print("Heading Error: {}").format(row_output[0])
    # print("Distance Error: {}").format(row_output[1] - row_output[2])
    # print("Right row distance: {}").format(row_output[1])
    # print("Left row distance: {}").format(row_output[2])
    
    # vel_corr(heading_err,row_dist_err)



    

def lidar():
    rospy.init_node('lidar_listener', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    rospy.Subscriber("assembled_cloud2", PointCloud2, callback)
    while not rospy.is_shutdown():
        rospy.spin()

   

        

if __name__ == '__main__':
    try:
        lidar()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown()