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
    robot = URDF.from_xml_file("/home/sunyuansong/ws_ur1.0/src/ur/universal_robot/ur_description/urdf/ur5_robot.urdf")
    tree = kdl_tree_from_urdf_model(robot)
    kdl_ee = KDLKinematics(robot, "base_link", "ee_link", tree)
    # path = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],[-0.00969267254689023, -0.16391365585935153, 0.1660385472166284, -0.0021808953906199044, -0.009733046071127049, -0.07908868621025233],[-0.01938534509378046, -0.32782731171870305, 0.3320770944332568, -0.004361790781239809, -0.019466092142254098, -0.15817737242050467],[-0.029078017640670695, -0.49174096757805463, 0.4981156416498852, -0.006542686171859714, -0.02919913821338115, -0.237266058630757],[-0.03877069018756092, -0.6556546234374061, 0.6641541888665136, -0.008723581562479618, -0.038932184284508196, -0.31635474484100934],[-0.04846336273445116, -0.8195682792967578, 0.8301927360831421, -0.010904476953099524, -0.048665230355635254, -0.3954434310512617],[-0.05815603528134139, -0.9834819351561093, 0.9962312832997704, -0.013085372343719428, -0.0583982764267623, -0.474532117261514],[-0.06784870782823162, -1.1473955910154607, 1.162269830516399, -0.015266267734339332, -0.06813132249788935, -0.5536208034717663],[-0.07754138037512184, -1.3113092468748122, 1.3283083777330271, -0.017447163124959236, -0.07786436856901639, -0.6327094896820187],[-0.08723405292201208, -1.475222902734164, 1.4943469249496557, -0.019628058515579142, -0.08759741464014345, -0.711798175892271]])
    path = np.array([[-2.87323242822756, -1.8249829451190394, -1.180469814931051, -1.97707444826235, 0.6349515914916992, -0.010372463856832326],[-2.877049265644405, -1.8465137375978982, -1.1539434335487786, -1.9770627529241127, 0.6339434867994368, -0.016583674286738515],[-2.8807589250949595, -1.8682897028745566, -1.1265735331345068, -1.9777732643334822, 0.632981452231817, -0.022636717709050604],[-2.8843658598414077, -1.8903388872836417, -1.0983080071211695, -1.9792257926984074, 0.63206292574343, -0.02853729269112589],[-2.8878742820789736, -1.9126932730020885, -1.0690866187688224, -1.9814445135481824, 0.6311855160467875, -0.03429084638369659],[-2.8912881765941543, -1.9353895079577417, -1.0388394400408987, -1.9844587976698231, 0.6303469886890712, -0.03990258090918392],[-2.8946113159234583, -1.958469902483213, -1.007484764038897, -1.9883042941862061, 0.6295452541048624, -0.045377466333899164],[-2.8978472743140036, -1.981983728833865, -0.9749263886281694, -1.9930243399343117, 0.6287783567032394, -0.0507202528677363],[-2.8850275324289565, -1.9859419698177783, -0.9651560758366882, -2.015877654463325, 0.631901271834973, -0.029625386544744982],[-2.8722660758318663, -1.9903058064426475, -0.954856322318892, -2.0385891656022483, 0.6352188214472001, -0.008810349223435897],[-2.859566829823508, -1.9950750426256594, -0.9440244359494987, -2.0611502921764164, 0.6387237512905414, 0.011712389690820483],[-2.8469336092270208, -2.0002499740113193, -0.9326560028627965, -2.0835550061564527, 0.6424085773110019, 0.03193170452546364],[-2.834370114466613, -2.0058314760575358, -0.9207447760821378, -2.105799819035956, 0.6462656227510721, 0.051837778827873215],[-2.8218799258988887, -2.0118211205083467, -0.9082824932187132, -2.127883824103361, 0.6502870596264422, 0.07142208318291919],[-2.8094664986672035, -2.018221301932451, -0.8952586731500074, -2.149808739936062, 0.6544649488514944, 0.0906773432026846],[-2.809466498035457, -2.0244668619958914, -0.8634765327296076, -2.175345321066214, 0.6544649490686528, 0.09067734417724742],[-2.8094664974037107, -2.031562448782526, -0.830208855760934, -2.201517412021446, 0.6544649492858112, 0.09067734515181014],[-2.809466496771964, -2.039591192171351, -0.7952796948877098, -2.2284178302790374, 0.6544649495029696, 0.09067734612637281],[-2.809466496140218, -2.0486568885220984, -0.7584700060533772, -2.2561618235358147, 0.6544649497201278, 0.09067734710093495],[-2.809466495508471, -2.0588915299063593, -0.7195020059555762, -2.284895183022547, 0.6544649499372862, 0.09067734807549767],[-2.809466494876725, -2.070466994405297, -0.6780148455077739, -2.314806879744604, 0.6544649501544445, 0.09067734905006047],[-2.8094664942449783, -2.0836138203517147, -0.6335255329935486, -2.346149367085604, 0.654464950371603, 0.09067735002462352]])
    print path

    joint_send = Joint()

    try:
        current_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        j_last_p = np.array(current_joint_pos)
        j_current_p = j_last_p
        listener = tf.TransformListener()
        rate = rospy.Rate(125)
        while not rospy.is_shutdown():
            path_raw = path.shape[0]
            path_col = path.shape[1]
            i = 0
            target = False
            while not target:
                if i == path_raw - 2 and np.linalg.norm(path[i+1, :]-j_current_p) < 0.001:
                    target = True
                joint_send.setJointPos(current_joint_pos)

                # projection matrix
                A = np.transpose([path[i+1, :]-j_last_p])
                print "A = ", A
                projection = np.dot(np.dot(A, np.linalg.inv(np.dot(np.transpose(A), A))), np.transpose(A))
                print "projection = ", projection

                # velocity to target
                j_v_target = (path[i+1, :]-j_last_p) / np.linalg.norm(A) * 2
                print np.linalg.norm(j_v_target)
                if np.linalg.norm(path[i+1, :]-j_current_p) < 0.001:
                    i = i+1
                    j_last_p = j_current_p
                print "j_v_target = ", j_v_target

                # ee Jacobian
                J_ee = kdl_ee.jacobian(current_joint_pos)
                J_I_ee = np.linalg.pinv(J_ee)

                # ee obstacle repulsion force
                try:
                    (trans_ee_o, rot_ee_o) = listener.lookupTransform('/tf1', '/ee_link', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
                vw_ee_o = calculateVelocity(trans_ee_o, 1)
                vw_ee = vw_ee_o
                j_v_ee = np.asarray(np.dot(J_I_ee, vw_ee))[0]

                # Project to the RRT path
                j_v_obstacle = np.asarray(np.dot(projection, j_v_ee))
                if np.dot(j_v_target, j_v_obstacle) >= 0:
                    j_v = j_v_target
                else :
                    j_v = j_v_obstacle + j_v_target
                    if np.dot(j_v, j_v_obstacle) >= 0:
                        j_v = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                print "j_v = ", j_v
                j_current_p += np.dot(j_v, 0.001)
                current_joint_pos = j_current_p.tolist()

                rate.sleep()
    except rospy.ROSInterruptException:
        pass
