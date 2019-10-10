#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
import urx

sys.path.insert(0, "/home/sunyuansong/ws_ur1.0/src/doa/doa_ur/scripts/hrl-kdl-indigo-devel/hrl_geom/src")
sys.path.insert(0, "/home/sunyuansong/ws_ur1.0/src/doa/doa_ur/scripts/hrl-kdl-indigo-devel/pykdl_utils/src")
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics
from urdf_parser_py.urdf import URDF

# import pdb
# pdb.set_trace()


class Joint(object):
    def __init__(self):
        self.joint = JointState()
        self.joint.header.seq = 0
        self.joint.header.stamp = rospy.Time.now()
        self.joint.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    def setJointPos(self, joint_pos):
        self.joint.header.stamp = rospy.Time.now()
        self.joint.position = joint_pos
        self.joint_pub.publish(self.joint)

    def setJointVel(self, joint_vel):
        self.joint.header.stamp = rospy.Time.now()
        self.joint.velocity = joint_vel
        self.joint_pub.publish(self.joint)

# def talker(joint_velocity):
#     pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
#     rate = rospy.Rate(125) # 125hz
#     trajectory = Float64MultiArray()
#     # qvel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     while not rospy.is_shutdown():
#         trajectory.data = joint_velocity        
#         print trajectory
#         pub.publish(trajectory)
#         rate.sleep()

def calculateVelocity(trans, force_type):
    # force type 0:attraction 1:repulsion
    trans_array = np.array(trans)
    length = np.linalg.norm(trans_array)
    trans_array = trans_array/length
    if force_type:
        if length <= 0.01:
            length = 0.01
        if length < 0.3:
            ee_v = trans_array/length
        else:
            ee_v = np.array([0, 0, 0])
        ee_w = np.array([0, 0, 0])
        vw_ee = np.append(ee_v, ee_w)
    else:
        if length <= 0.01:
            ee_v = np.array([0, 0, 0])
        else:
            ee_v = trans_array * 1.5
        ee_w = np.array([0, 0, 0])
        vw_ee = np.append(ee_v, ee_w)
    return vw_ee


if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_avoidance')
    rob = urx.Robot("192.168.31.53")

    pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
    joint_velocity = Float64MultiArray()
    joint_velocity.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    robot = URDF.from_xml_file("/home/sunyuansong/ws_ur1.0/src/ur/universal_robot/ur_description/urdf/ur5_robot.urdf")
    tree = kdl_tree_from_urdf_model(robot)
    kdl_ee = KDLKinematics(robot, "base_link", "ee_link", tree)

    joint_send = Joint()

    try:
        listener = tf.TransformListener()
        rate = rospy.Rate(125)
                
        while not rospy.is_shutdown():
            current_joint_pos = rob.getj()
            joint_send.setJointPos(current_joint_pos)
            pub.publish(joint_velocity)
            
            # ee Jacobian
            J_ee = kdl_ee.jacobian(current_joint_pos)
            J_I_ee = np.linalg.pinv(J_ee)
            # ee obstacle repulsion force
            try:
                (trans_ee_o, rot_ee_o) = listener.lookupTransform('/tf1', '/ee_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            vw_ee_o = calculateVelocity(trans_ee_o, 1)
            # # ee target attraction force
            # try:
            #     (trans_ee_t, rot_ee_t) = listener.lookupTransform('/link_6', '/obstacle', rospy.Time(0))
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     continue
            # vw_ee_t = calculateVelocity(trans_ee_t, 0)
            # vw_ee = vw_ee_o + vw_ee_t
            vw_ee = vw_ee_o
            j_v_ee = np.asarray(np.dot(J_I_ee, vw_ee))[0]
            joint_velocity.data = j_v_ee.tolist()

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
