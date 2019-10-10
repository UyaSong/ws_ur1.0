#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
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

def calculateVelocity(trans, force_type):
    # force type 0:attraction 1:repulsion
    trans_array = np.array(trans)
    length = np.linalg.norm(trans_array)
    trans_array = trans_array/length
    if force_type:
        if length <= 0.05:
            length = 0.05
        if length < 0.3:
            ee_v = trans_array/length
        else:
            ee_v = np.array([0, 0, 0])
        ee_w = np.array([0, 0, 0])
        ee_vw = np.append(ee_v, ee_w)
    else:
        if length <= 0.01:
            ee_v = np.array([0, 0, 0])
        else:
            ee_v = trans_array * 1.5
        ee_w = np.array([0, 0, 0])
        ee_vw = np.append(ee_v, ee_w)
    return ee_vw


def callback(vector, args):
    J_I_ee = args[0]
    joint_velocity = args[1]
    pub = args[2]
    j_v_target = args[3]
    projection = args[4]
    vector_list = [vector.x, vector.y, vector.z]
    # print vector_list
    ee_v_o = calculateVelocity(vector_list, 1)
    ee_v = ee_v_o
    print "ee_v = ", ee_v
    j_v_ee = np.asarray(np.dot(J_I_ee, ee_v))[0]

    # Project to the RRT path
    j_v_obstacle = np.asarray(np.dot(projection, j_v_ee))
    if np.dot(j_v_obstacle, j_v_target) >= 0:
        j_v = j_v_target
    else:
        j_v = j_v_obstacle + j_v_target
        if np.dot(j_v, j_v_target) < 0:
            j_v = 0
    print "j_v_target = ", j_v_target
    print "j_v norm = ", np.linalg.norm(j_v)

    joint_velocity.data = j_v_target.tolist()
    pub.publish(joint_velocity)

if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_avoidance')
    rob = urx.Robot("192.168.31.53")

    pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
    joint_velocity = Float64MultiArray()
    joint_velocity.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    robot = URDF.from_xml_file("/home/sunyuansong/ws_ur1.0/src/ur/universal_robot/ur_description/urdf/ur5_robot.urdf")
    tree = kdl_tree_from_urdf_model(robot)
    kdl_ee = KDLKinematics(robot, "base_link", "ee_link", tree)
    path = np.array([[-2.7702205816852015, -1.5404894987689417, -1.396792236958639, -2.172868076954977, 0.6689021587371826, 0.1500646024942398],[-2.7779733606204635, -1.5631945259854227, -1.376631030035783, -2.161242432635435, 0.6659336246331153, 0.1384986324347235],[-2.7854098114557204, -1.586040090122865, -1.3556978724602555, -2.15053506798668, 0.663147783807624, 0.1273231890420074],[-2.7925488723747782, -1.6090350245768577, -1.333975498894453, -2.1407429404087894, 0.6605307979039394, 0.11652038867262628],[-2.799408003740675, -1.6321900230761448, -1.3114437280755946, -2.1318644959556603, 0.6580700811825718, 0.10607330556839321],[-2.8060033270798574, -1.6555177397758833, -1.2880791149233783, -2.1238999428406875, 0.6557541631923107, 0.09596594110172409],[-2.812349750875677, -1.6790329846362015, -1.2638544703200885, -2.11685152522649, 0.6535725746294581, 0.08618316674954404],[-2.8184610824476195, -1.7027529631901634, -1.238738291049107, -2.110723840903408, 0.6515157464053873, 0.07671067084630893],[-2.8243501276792427, -1.7266975726652647, -1.21269407548017, -2.105524215744662, 0.6495749202201858, 0.06753490884197193],[-2.830028780106941, -1.750889770037221, -1.185679492937864, -2.101263151781344, 0.6477420691818493, 0.05864305681654434],[-2.835508100670302, -1.7753560326546547, -1.1576453640512174, -2.0979548712513085, 0.6460098272146492, 0.050022968024695844],[-2.840798389246133, -1.8001269392437547, -1.128534394369539, -2.09561798676191, 0.644371426173283, 0.041663132262486365],[-2.845909248936743, -1.8252379094135447, -1.0982795819955788, -2.0942763388822936, 0.6428206397263867, 0.03355263786446537],[-2.85084964395417, -1.8507301548674322, -1.0668021885313237, -2.0939600588130314, 0.6413517331982175, 0.025681136153711866],[-2.8556279518321444, -1.876651918032433, -1.034009115741909, -2.094706938132963, 0.6399594186642572, 0.018038808180107932],[-2.8602520106035, -1.9030601081519676, -0.9997894588405175, -2.096564224764906, 0.638638814688066, 0.010616333593766892],[-2.864729161500112, -1.9300224986129613, -0.9640098954382446, -2.099591022414243, 0.6373854101653144, 0.0034048615110935333],[-2.869066287663022, -1.9576207358052629, -0.9265083890624642, -2.103861564329809, 0.6361950318085119, -0.003604016759279302],[-2.8732698492905615, -1.9859545538551457, -0.887085386267818, -2.109469788057697, 0.6350638148642762, -0.010418293254339583],[-2.877345915600505, -2.0151478387118855, -0.8454911676811259, -2.116535907392525, 0.6339881767053083, -0.01704556727346089],[-2.8813001939372467, -2.045357635664723, -0.8014070751713315, -2.1252161652733164, 0.6329647929829124, -0.023493067781021757],[-2.8680435466257164, -2.051262013052958, -0.7882009190496339, -2.1498771003031267, 0.6364788412937721, -0.001935554839558837],[-2.854856170917301, -2.0576921381809354, -0.7742321390259167, -2.1744705482019375, 0.6401928091964564, 0.019300631001244215],[-2.841742308580778, -2.064656932128457, -0.7594775278840314, -2.1989999063833983, 0.6440981592686376, 0.04020351120647857],[-2.828706060408939, -2.0721675930725167, -0.7439084586105804, -2.223473243843513, 0.6481861566474652, 0.06076266291509371],[-2.8157513828240353, -2.080238010460361, -0.7274901072221323, -2.247903600169431, 0.6524479116172383, 0.08096915766079855],[-2.802882081980222, -2.088885329733497, -0.7101803470683551, -2.272309480692177, 0.6568744276657236, 0.10081551703779515],[-2.790101808554478, -2.098130657053061, -0.6919283499734206, -2.296715494924451, 0.6614566472430479, 0.12029565836227883],[-2.7774140532341267, -2.1079999556807634, -0.6726727858673788, -2.3211531967152763, 0.6661854949176719, 0.139404832064067],[-2.76482214290338, -2.118525209219263, -0.6523394636819265, -2.3456622098345634, 0.6710519176920081, 0.15813955249730116],[-2.752329237525987, -2.1297459636268057, -0.6308381799905473, -2.370291762203438, 0.6760469223067538, 0.17649752377950126],[-2.7399383277160547, -2.1417114189004463, -0.6080584191882004, -2.3951028155826606, 0.6811616094246794, 0.19447756216033518]])

    joint_send = Joint()

    try:
        listener = tf.TransformListener()
        current_joint_pos = rob.getj()
        joint_send.setJointPos(current_joint_pos)
        j_last_p = np.array(current_joint_pos)
        j_current_p = j_last_p
        path_raw = path.shape[0]
        path_col = path.shape[1]
        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            i = 0
            target = False
            while not target:
                if i == path_raw - 1 and np.linalg.norm(path[i, :]-j_current_p) < 0.01:
                    target = True
                    continue
                if np.linalg.norm(path[i, :]-j_current_p) < 0.01:
                    i = i+1
                    j_last_p = j_current_p

                current_joint_pos = rob.getj()
                joint_send.setJointPos(current_joint_pos)
                j_current_p = np.array(current_joint_pos)
                print "target pos = ", path[i, :]
                print "current pos = ", j_current_p
                print "last pos = ", j_last_p

                if np.dot(path[i, :]-j_last_p, path[i, :]-j_current_p) < 0:
                    i = i+1
                    j_last_p = j_current_p

                # projection matrix
                A = np.transpose([path[i, :]-j_last_p])
                # print "A = ", A
                projection = np.dot(np.dot(A, np.linalg.inv(np.dot(np.transpose(A), A))), np.transpose(A))
                # print "projection = ", projection

                # velocity to target
                j_v_target = (path[i, :]-j_last_p) / np.linalg.norm(A) * 0.1
                # print np.linalg.norm(j_v_target)
                # print "j_v_target = ", j_v_target

                # ee Jacobian
                J_ee = kdl_ee.jacobian(current_joint_pos)
                J_I_ee = np.linalg.pinv(J_ee)

                ## test
                joint_velocity.data = j_v_target.tolist()
                joint_velocity.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                pub.publish(joint_velocity)

                # vector_list = [dise_listener.dist_vector.x, dise_listener.dist_vector.y, dise_listener.dist_vector.z]
                # rospy.Subscriber("/kinect_merge/vector_closest_frame", Vector3, callback, (J_I_ee, joint_velocity, pub, j_v_target, projection))

                rate.sleep()
    except rospy.ROSInterruptException:
        pass
