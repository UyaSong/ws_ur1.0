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
import socket

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


def calculateAcceleration(trans, force_type):
    # force type 0:attraction 1:repulsion
    trans_array = np.array(trans)
    length = np.linalg.norm(trans_array)
    trans_array = trans_array/length
    if force_type:
        if length <= 0.05:
            length = 0.05
        if length < 0.1:
            ee_va = trans_array/length*0.005
        else:
            ee_va = np.array([0, 0, 0])
        ee_wa = np.array([0, 0, 0])
        ee_a = np.append(ee_va, ee_wa)
    else:
        if length <= 0.001:
            ee_va = np.array([0, 0, 0])
        else:
            ee_va = trans_array * 1.5
        ee_wa = np.array([0, 0, 0])
        ee_a = np.append(ee_va, ee_wa)
    return ee_a


class doa():
    def __init__(self):
        HOST = "192.168.31.53"    # The remote host
        PORT = 30003        # The same port as used by the server
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        rob = urx.Robot("192.168.31.53")
        self.pub = rospy.Publisher('/joint_group_vel_controller/command', Float64MultiArray, queue_size=10)
        self.joint_velocity = Float64MultiArray()
        self.joint_velocity.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.current_joint_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # rospy.Subscriber("/joint_states", Vector3, self.jointsub_callback)
        robot = URDF.from_xml_file("/home/sunyuansong/ws_ur1.0/src/ur/universal_robot/ur_description/urdf/ur5_robot.urdf")
        tree = kdl_tree_from_urdf_model(robot)
        kdl_ee = KDLKinematics(robot, "base_link", "ee_link", tree)
        path = np.array([
            [-2.9114747683154505, -1.7126920858966272, -1.6692927519427698, -2.6685803572284144, 0.50324946641922, 1.5052677392959595]
            ,[-2.9160730071217382, -1.734746453609111, -1.643601713083201, -2.67026705583814, 0.4987766846534723, 1.5030511334837948]
            ,[-2.920489510780279, -1.7567648028408513, -1.617421449852772, -2.672520891730043, 0.4944826892441575, 1.5008874640603136]
            ,[-2.924734930942239, -1.7787650502057728, -1.5907361053881952, -2.6753384931391233, 0.49035693830746946, 1.4987747519624455]
            ,[-2.928819098831518, -1.8007656739865954, -1.5635277740654314, -2.678718022055244, 0.48638970471395737, 1.4967111183683868]
            ,[-2.9327511007476494, -1.8227857991190624, -1.5357763244985085, -2.6826592702165843, 0.4825720008088957, 1.4946947830186077]
            ,[-2.936539347835964, -1.8448453369690738, -1.5074591497417333, -2.687163766748072, 0.4788955090303036, 1.4927240572000342]
            ,[-2.940191638103886, -1.866965146123628, -1.478550871880363, -2.692234910516189, 0.4753525202447543, 1.4907973373808767]
            ,[-2.9437152116773424, -1.8891672190378495, -1.4490229910702634, -2.6978781322926504, 0.47193587881118104, 1.4889130994209305]
            ,[-2.9471168001473074, -1.9114749006640708, -1.4188434663957348, -2.704101093226906, 0.4686389335262522, 1.4870698932925228]
            ,[-2.9504026707357363, -1.9339131468979338, -1.387976212343751, -2.710913927989288, 0.46545549372517847, 1.4852663382559887]
            ,[-2.95357866590844, -1.9565088329735807, -1.3563804899022647, -2.7183295434401087, 0.46237978991310763, 1.4835011184409488]
            ,[-2.9566502389765374, -1.9792911250621832, -1.3240101647860796, -2.7263639870620797, 0.4594064383878226, 1.4817729787908933]
            ,[-2.95962248615529, -2.002291932624844, -1.2908127963475695, -2.735036904042957, 0.45653040938700595, 1.480080721333909]
            ,[-2.9625001754871843, -2.025546465067786, -1.2567285082473327, -2.744372108380805, 0.4537469983550079, 1.478423201746972]
            ,[-2.9652877729832903, -2.049093924747352, -1.2216885742704982, -2.7543983025741685, 0.4510517999766838, 1.4767993261850827]
            ,[-2.9679894662917388, -2.072978380627485, -1.1856136271701683, -2.7651499937087345, 0.4484406846708639, 1.4752080483499361]
            ,[-2.970609186163363, -2.0972498849005716, -1.1484113609422386, -2.7766686732212826, 0.44590977727464937, 1.4736483667757079]
            ,[-2.9731506259511997, -2.1219659219096503, -1.1099735406875264, -2.789004356843108, 0.4434554376829445, 1.4721193223120554]
            ,[-2.9756172593517607, -2.1471933202430575, -1.070172047775284, -2.8022176261340794, 0.44107424323628797, 1.4706199957866504]
            ,[-2.978012356571096, -2.1730108244204356, -1.0288535515919173, -2.8163823839010895, 0.4387629726748473, 1.4691495058314532]
            ,[-2.9803389990770635, -2.1995126292139022, -0.9858321772185528, -2.8315896511102134, 0.4365185914979311, 1.4677070068586722]
            ,[-2.980338999690519, -2.1986747547170724, -0.9640149700494741, -2.854244732430145, 0.4365185909060915, 1.467707006476896]
            ,[-2.980339000303974, -2.1984231037000894, -0.9412488821662535, -2.8772624709843715, 0.4365185903142518, 1.4677070060951198]
            ,[-2.9803390009174295, -2.1987868711684007, -0.9174745864184775, -2.9006729989178592, 0.4365185897224119, 1.4677070057133434]
            ,[-2.980339001530885, -2.199799804477071, -0.8926229827666841, -2.9245116689150055, 0.4365185891305724, 1.4677070053315673]
            ,[-2.98033900214434, -2.2015012157109233, -0.8666131181357992, -2.948820121966061, 0.43651858853873265, 1.4677070049497911]
            ,[-2.9803390027577956, -2.2039373574560144, -0.8393493353370614, -2.973647762673731, 0.43651858794689286, 1.4677070045680147]
            ,[-2.9803390033712507, -2.2071632556780587, -0.8107174706634052, -2.999053728779366, 0.43651858735505306, 1.4677070041862386]
            ,[-2.980339003984706, -2.211245201576228, -0.780579679833301, -3.0251095733653237, 0.4365185867632133, 1.4677070038044622]
            ,[-2.9803390045981613, -2.2162642190426447, -0.7487672330079576, -3.051903002378274, 0.4365185861713735, 1.4677070034226858]
            ,[-2.9803390052116168, -2.222321021714748, -0.7150702086612833, -3.079543223706868, 0.4365185855795337, 1.4677070030409094]
            ,[-2.980339005825072, -2.229543328192302, -0.6792222777125801, -3.10816884783204, 0.4365185849876939, 1.4677070026591332]
            ,[-2.9778941835766273, -2.194253348383131, -0.7378626367008178, -3.0861896122236603, 0.43887770286477806, 1.4692208564930036]
            ,[-2.9753737440067387, -2.160931199444495, -0.7920966689259088, -3.066675351334529, 0.4413107007672305, 1.4707656818342705]
            ,[-2.9727740856336067, -2.1291645995322117, -0.8427723254186107, -3.0491911748194607, 0.4438211246474617, 1.4723424757188697]
            ,[-2.97009139465559, -2.0986610150774156, -0.8904881674730437, -3.033431851621574, 0.44641273104461776, 1.4739522663007183]
            ,[-2.967321607021797, -2.069202051030464, -0.9356881759723472, -3.0191726235445935, 0.44908952416777936, 1.475596128241982]
            ,[-2.964460387506783, -2.0406192562944545, -0.978712100918531, -3.00624305174531, 0.45185577666689763, 1.477275185501429]
            ,[-2.961503106602162, -2.0127792047765882, -1.0198265013605927, -2.9945108973079844, 0.4547160525734686, 1.478990614363454]
            ,[-2.9584448149770646, -1.9855738473505298, -1.0592448136430805, -2.983871702680188, 0.4576752326583647, 1.480743646719441]
            ,[-2.95528021520867, -1.958914023907205, -1.0971408424034022, -2.9742417903669973, 0.4607385425043325, 1.4825355736243746]
            ,[-2.9520036304371677, -1.932724955896718, -1.1336581298879727, -2.9655534029152, 0.4639115836372577, 1.48436774915658]
            ,[-2.9486089695465236, -1.9069430271473258, -1.1689166444685062, -2.957751235525545, 0.4672003681130093, 1.4862415946130567]
            ,[-2.9450896884110436, -1.881513429655188, -1.2030176694314982, -2.950789903532137, 0.47061135701780554, 1.4881586030777405]
            ,[-2.9414387466758583, -1.8563884061937164, -1.2360474501306127, -2.944632054816675, 0.4741515034115952, 1.490120344405496]
            ,[-2.9376485594548125, -1.8315259146162437, -1.2680799639495905, -2.939246937842743, 0.4778283003282391, 1.4921284706709776]
            ,[-2.9337109432291193, -1.8068885963873673, -1.299179057493202, -2.9346092983640664, 0.4816498345459908, 1.494184722138876]
            ,[-2.9296170551112772, -1.782442968675726, -1.3294001188192133, -2.9306985176679485, 0.48562484696014696, 1.4962909338207795]
            ,[-2.9253573244970568, -1.7581587834334709, -1.3587914023580576, -2.927497931288919, 0.4897628005308649, 1.498449042694184]
            ,[-2.920921375958832, -1.7340085130237541, -1.3873950905522952, -2.9249942846061767, 0.494073956947976, 1.5006610956714979]
            ,[-2.9162979420299253, -1.7099669329840228, -1.415248153256843, -2.923177293702737, 0.498569463357435, 1.5029292584215432]
            ,[-2.91147476428397, -1.6860107801823152, -1.4423830499225143, -2.9220392882128894, 0.5032614507386876, 1.50525582516379]
        ])
        joint_send = Joint()
        current_joint_pos = rob.getj()
        joint_send.setJointPos(current_joint_pos)
        j_last_p = np.array(current_joint_pos)
        j_current_p = j_last_p

        j_current_v = 0.0

        path_raw = path.shape[0]
        # path_col = path.shape[1]
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            i = 0
            target = False
            rate = rospy.Rate(125)
            while not target:
                if i == path_raw - 1 and np.linalg.norm(path[i, :]-j_current_p) < 0.01:
                    target = True
                    continue
                if np.linalg.norm(path[i, :]-j_current_p) < 0.01:
                    i = i+1
                    j_last_p = j_current_p
                if np.dot(path[i, :]-j_last_p, path[i, :]-j_current_p) < 0:
                    i = i+1
                    j_last_p = j_current_p

                current_joint_pos = rob.getj()
                joint_send.setJointPos(current_joint_pos)
                j_current_p = np.array(current_joint_pos)
                # print "target pos = ", path[i, :]
                # print "current pos = ", j_current_p
                # print "last pos = ", j_last_p

                # projection matrix
                A = np.transpose([path[i, :]-j_last_p])
                self.projection = np.dot(np.dot(A, np.linalg.inv(np.dot(np.transpose(A), A))), np.transpose(A))

                # velocity to target
                self.j_v_target = (path[i, :]-j_last_p) / np.linalg.norm(A)

                # ee Jacobian
                J_ee = kdl_ee.jacobian(current_joint_pos)
                self.J_I_ee = np.linalg.pinv(J_ee)

                # ee obstacle repulsion force
                try:
                    (trans_ee_o, rot_ee_o) = listener.lookupTransform('/tf1', '/ee_link', rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    trans_ee_o = [100, 100, 100]
                    # continue
                ee_v_o = calculateVelocity(trans_ee_o, 1)
                ee_v = ee_v_o
                j_v_ee = np.asarray(np.dot(self.J_I_ee, ee_v))[0]
                ee_a_o = calculateAcceleration(trans_ee_o, 1)
                ee_a = ee_a_o
                j_a_ee = np.asarray(np.dot(self.J_I_ee, ee_a))[0]

                # Project to the RRT path
                j_v_obstacle = np.asarray(np.dot(self.projection, j_v_ee))
                j_a_obstacle = np.asarray(np.dot(self.projection, j_a_ee))
                j_a_t_norm = 0.005
                if np.dot(j_a_obstacle, self.j_v_target) >= 0:
                    j_a_o_norm = 0
                else:
                    j_a_o_norm = np.linalg.norm(j_a_obstacle)*0.005
                j_current_v = j_current_v + j_a_t_norm - j_a_o_norm
                # if np.dot(j_v_obstacle, self.j_v_target) >= 0:
                #     j_v = self.j_v_target
                # else:
                #     j_v = j_v_obstacle + self.j_v_target
                #     if np.dot(j_v, self.j_v_target) < 0:
                #         j_v = 0
                # print "j_v_target = ", self.j_v_target
                # print "j_v norm = ", np.linalg.norm(j_v)
                if j_current_v < 0:
                    j_current_v = 0
                if j_current_v > 0.05:
                    j_current_v = 0.05
                j_v = j_current_v * self.j_v_target
                j_v_list = j_v.tolist()
                # strL = b"speedj([0.2,0,0,0,0,0],0.4,0.008)\n"
                strL = b"speedj([%f,%f,%f,%f,%f,%f],0.4,0.01)\n"%(j_v_list[0], j_v_list[1], j_v_list[2], j_v_list[3], j_v_list[4], j_v_list[5])
                print strL
                s.send(strL)
                # self.pub.publish(self.joint_velocity)
                # print self.joint_velocity

                rate.sleep()
    # def jointsub_callback(self, JointState):


if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_avoidance')
    doa_ur = doa()
    rospy.spin()
      