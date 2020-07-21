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
import scipy.io as scio


rodclimbing_minjoints=[-0.1, -0.5]
rodclimbing_maxjoints=[1.0, 3/4.0*math.pi]
arm_minjoints=[-1/6*math.pi,-175/180.0*math.pi,-175/180.0*math.pi,-175/180.0*math.pi,-175/180.0*math.pi,-175/180.0*math.pi]
arm_maxjoints=[175/180.0*math.pi,175/180.0*math.pi,175/180.0*math.pi,175/180.0*math.pi,175/180.0*math.pi,175/180.0*math.pi]

def talker():
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10000) 

    file_name="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_description/urdf/base/paintingrobot_description_witharm.urdf"
    marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    visualization_num=1
    N=5000000
    # N=2
    workspace_points=np.zeros((N,4))
    q=np.zeros(11)
    arm_q=np.zeros(6)

    while not rospy.is_shutdown():
        robot = URDF.from_xml_file(file_name)
        kdl_kin = KDLKinematics(robot,"base_link", "tool0")
        tree = kdl_tree_from_urdf_model(robot)
        chain = tree.getChain("base_link", "tool0")

        q[0]=0.0
        q[1]=0.0
        q[2]=0.0

        q[3]=rodclimbing_minjoints[0]+(rodclimbing_maxjoints[0]-rodclimbing_minjoints[0])*random.random()
        q[4]=rodclimbing_minjoints[1]+(rodclimbing_maxjoints[1]-rodclimbing_minjoints[1])*random.random()  

        q[5]=arm_minjoints[0]+(arm_maxjoints[0]-arm_minjoints[0])*random.random()
        q[6]=arm_minjoints[1]+(arm_maxjoints[1]-arm_minjoints[1])*random.random()

        q[7]=arm_minjoints[2]+(arm_maxjoints[2]-arm_minjoints[2])*random.random()
        q[8]=arm_minjoints[3]+(arm_maxjoints[3]-arm_minjoints[3])*random.random()

        q[9]=arm_minjoints[4]+(arm_maxjoints[4]-arm_minjoints[4])*random.random()
        q[10]=arm_minjoints[5]+(arm_maxjoints[5]-arm_minjoints[5])**random.random()

        # q[7]=math.pi/2
        # q[8]=math.pi/2
        # q[9]=math.pi/2
        # q[10]=-175/180.0*math.pi

        pose = kdl_kin.forward(q)  

        for i in range(len(arm_q)):
            arm_q[i]=q[i+5]
        
        kdl_kin1 = KDLKinematics(robot,"aubo_baselink", "wrist3_Link")
        J = kdl_kin1.jacobian(arm_q)
        rank1=np.linalg.matrix_rank(J)
        print("rank is:",rank1)
        manipulabilty_mat=np.dot(J,J.T)
        rank2=np.linalg.matrix_rank(manipulabilty_mat)
        print("rank2 is:",rank2)
        manipulability_value=math.sqrt(np.linalg.det(manipulabilty_mat))

        k=1
        q_formula_value=1
        for i in range(len(arm_q)):
            # print("q_formula_value is:",q_formula_value)
            q_formula_value=q_formula_value*(arm_q[i]-arm_minjoints[i])*(arm_maxjoints[i]-arm_q[i])/(arm_maxjoints[i]-arm_minjoints[i])**2
        L_value=1-math.exp(-k*q_formula_value)

        dexterity_value=manipulability_value

        workspace_points[visualization_num,0]=pose[0,3]
        workspace_points[visualization_num,1]=pose[1,3]
        workspace_points[visualization_num,2]=pose[2,3]
        workspace_points[visualization_num,3]=dexterity_value

        print("arm_q is: ",arm_q)
        print("pose is:",pose)
        print("jacobian is: ", J)
        # print("manipulabilty_mat is: ",manipulabilty_mat)
        # print("manipulability_value is: ", manipulability_value)
        # print("L_value is: ",L_value)
        print("dexterity_value is:", dexterity_value)
        print("visualization_num is: ",visualization_num)
        visualization_num=visualization_num+1

        if visualization_num>=N:
            break
        # rate.sleep()
    scio.savemat('/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_workspace_computation/scripts/data4.mat',{'workspace_points':workspace_points})


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass