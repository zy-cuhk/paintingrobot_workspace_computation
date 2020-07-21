#!/usr/bin/env python

import rospy
from std_msgs.msg import String, ColorRGBA

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from geometry_msgs.msg import PoseStamped, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from numpy.matlib import *
import math 
import random

def targetpositions_visualization(waypoint, frame, num, scale, color):
    marker1 = Marker()
    marker1.header.frame_id = frame
    marker1.type = Marker.CUBE
    marker1.action = Marker.ADD

    marker1.scale.x = scale[0]
    marker1.scale.y = scale[1]
    marker1.scale.z = scale[2]
    marker1.ns = 'targeposition'
    marker1.id = num
    marker1.color.r = color[0]
    marker1.color.g = color[1]
    marker1.color.b = color[2]
    marker1.color.a = 1.0
    marker1.lifetime = rospy.Duration()

    marker1.pose.position.x = waypoint[0]
    marker1.pose.position.y = waypoint[1]
    marker1.pose.position.z = waypoint[2]
    marker1.pose.orientation.w = waypoint[3]
    marker1.pose.orientation.x = waypoint[4]
    marker1.pose.orientation.y = waypoint[5]
    marker1.pose.orientation.z = waypoint[6]
    return marker1, num



def talker():
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10000) 

    file_name="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_description/urdf/base/paintingrobot_description_witharm.urdf"
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    visualization_num=1
    N=500000
    q=np.zeros(11)

    while not rospy.is_shutdown():
        robot = URDF.from_xml_file(file_name)
        kdl_kin = KDLKinematics(robot,"base_link", "tool0")
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain("base_link", "tool0")

        q[0]=0.0
        q[1]=0.0
        q[2]=0.0

        q[3]=-0.1+1.1*random.random()
        q[4]=-0.5+(0.5+3/4.0*math.pi)*random.random()  

        q[5]=-1/6*math.pi+(1/6+175/180.0)*math.pi*random.random()
        q[6]=-175/180.0*math.pi+2*175/180.0*math.pi*random.random()
        q[7]=-175/180.0*math.pi+2*175/180.0*math.pi*random.random()
        q[8]=-175/180.0*math.pi+2*175/180.0*math.pi*random.random()
        q[9]=-175/180.0*math.pi+2*175/180.0*math.pi*random.random()
        q[10]=-175/180.0*math.pi+2*175/180.0*math.pi*random.random()

        # print("the joints are:",q)
        # print("the random number is:",175/180.0*math.pi*random.random())

        pose = kdl_kin.forward(q)  
        # print "pose is:", pose[0,3], pose[1,3], pose[2,3]

        frame = 'base_link'
        mobileplatform_targepositions=np.array([pose[0,3], pose[1,3], pose[2,3],1.0,0.0,0.0,0.0])
        scale1=np.array([0.05,0.05,0.05])
        color1=np.array([0.0,1.0,0.0])
        marker1,visualization_num=targetpositions_visualization(mobileplatform_targepositions, frame, visualization_num, scale1, color1)
        marker_pub.publish(marker1)
        visualization_num=visualization_num+1
        print("visualization_num is:",visualization_num)

        if visualization_num>N:
            break
        rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass