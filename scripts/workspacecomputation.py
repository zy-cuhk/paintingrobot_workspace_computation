
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import sys
# import rospy
# from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# from pykdl_utils.kdl_kinematics import KDLKinematics
# import numpy as np
# from numpy.matlib import *

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# def robot_fk_computation(file_name):
#     robot = URDF.from_xml_file(file_name)
#     kdl_kin = KDLKinematics(robot,"base_link", "wrist3_Link")
#     tree = kdl_tree_from_urdf_model(robot)
#     chain = tree.getChain("base_link", "wrist3_Link")
#     q=np.ones(11)
#     print("q is:",q)
#     pose = kdl_kin.forward(q)  
#     print("pose is:",pose)
#     return 0



# class UR_robot:
#     def __init__(self):
#         self.robot = self.init_robot("/home/zy/catkin_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
#         self.kdl_kin = KDLKinematics(self.robot, "base_link", "ee_link")
#         self.tree = kdl_tree_from_urdf_model(self.robot)
#         self.chain = self.tree.getChain("base_link", "ee_link")
#         self.safe_q = [1.3189744444444444, -2.018671111111111, 1.8759755555555557, 2.7850055555555557, 0.17444444444444443, 3.7653833333333337]
#         self.q = self.safe_q

#     def init_robot(self, filename):
#         robot = URDF.from_xml_file(filename)
#         return robot

#     def set_q(self, q_list):
#         self.q = q_list

#     def get_fk_pose(self):
#         q = self.q
#         pose = self.kdl_kin.forward(q)  
#         return pose

#     def get_chain(self):
#         self.tree = kdl_tree_from_urdf_model(self.robot)
#         self.chain = self.tree.getChain("base_link", "ee_link")

# def main():
#     # z_height=0.5
#     # radius=0.2
#     # ur5 = UR_robot()
#     # aubo_q=[1.50040841e-03, -2.83640237e+00, 1.53798406e+00, 1.29841831e+00, 1.50040840e-03, 3.14159265e+00]
#     # ur5.set_q(aubo_q)
#     # pose=ur5.get_fk_pose()
#     # pose[0,3]=-0.54
#     # pose[2,3]=0.51
#     # print("pose is:",pose)
#     # q_ik=ur5.get_ik_pose(pose)
#     # print(q_ik*180/pi)
#     file_name="/home/zy/catkin_ws/src/paintingrobot_related/paintingrobot_underusing/paintingrobot_description/urdf/base/paintingrobot_description_witharm.urdf.xacro"

#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         robot_fk_computation(file_name)
#         rate.sleep()




# if __name__ == "__main__":
#     main()

















