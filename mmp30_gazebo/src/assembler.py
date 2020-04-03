#!/usr/bin/env python
import rospy
from laser_assembler.srv import *
from sensor_msgs.msg import PointCloud2

rospy.init_node("test")
rospy.wait_for_service("assemble_scans2")

assemble_scans = rospy.ServiceProxy("assemble_scans2", AssembleScans2)
pub = rospy.Publisher("/laser_pointcloud",PointCloud2, queue_size=1)
r = rospy.Rate(1)


while not rospy.is_shutdown():
    try:
        resp = assemble_scans(rospy.Time(0,0), rospy.Time.now())
        print "Got cloud with %u points" % len(resp.cloud.data)
        pub.publish(resp.cloud)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e 

    r.sleep()