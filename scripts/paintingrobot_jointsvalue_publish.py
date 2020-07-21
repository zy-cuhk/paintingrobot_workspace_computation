#!/usr/bin/env python

import rospy, sys
import math
import tf
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf_conversions import transformations
from math import pi

class Renovationrobot_joints_pub():
    def __init__(self):
        self.mobile_platform_joints_value=[0.0,0.0,0.0]
        self.jackup_mechanism_joints_value=[0.0,0.0]
        self.aubo_joints_value=[0.0,0.0,0.0,0.0,0.0,0.0]
        self.paintingrobot_joints_pub=rospy.Publisher('/joint_states', JointState, queue_size=10)

    def obtain_paintingrobot_states(self):
        paintingrobot_joints=JointState()
        paintingrobot_joints.header.stamp=rospy.Time.now()
        paintingrobot_joints.name = []
        paintingrobot_joints.position = []
        paintingrobot_joints.velocity = []
        paintingrobot_joints.effort = []

        paintingrobot_joints.name.append('base_joint1')
        paintingrobot_joints.position.append(0.0)
        paintingrobot_joints.name.append('base_joint2')
        paintingrobot_joints.position.append(0.0)
        paintingrobot_joints.name.append('mobilebase_joint')
        paintingrobot_joints.position.append(0.0)
        print("the received angle is:",self.mobile_platform_joints_value[2])

        paintingrobot_joints.name.append('rodclimbing_joint1')
        paintingrobot_joints.position.append(self.jackup_mechanism_joints_value[0])
        paintingrobot_joints.name.append('rodclimbing_joint2')
        paintingrobot_joints.position.append(self.jackup_mechanism_joints_value[1])

        paintingrobot_joints.name.append('shoulder_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[0])
        paintingrobot_joints.name.append('upperArm_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[1])
        paintingrobot_joints.name.append('foreArm_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[2])
        paintingrobot_joints.name.append('wrist1_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[3])
        paintingrobot_joints.name.append('wrist2_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[4])
        paintingrobot_joints.name.append('wrist3_joint')
        paintingrobot_joints.position.append(self.aubo_joints_value[5])

        self.paintingrobot_joints_pub.publish(paintingrobot_joints)
        
if __name__ == '__main__':
    rospy.init_node('paintingrobot_jointsvalue_publish', anonymous=True)
    rate = rospy.Rate(10.0)
    Renovationrobot=Renovationrobot_joints_pub()
    while not rospy.is_shutdown():
        Renovationrobot.obtain_paintingrobot_states()
        rate.sleep()









