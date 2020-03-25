#!/usr/bin/python
from . import utils
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import tf
import json
import rospy
from oven.srv import *
import time
import copy

eps = 1e-5
T_identity = [[eps, eps, eps], [eps, eps, eps, 1.0]]

def tweak(T_tool_to_obj_seq, T_tweak_seq):
    n = len(T_tweak_seq)
    assert len(T_tool_to_obj_seq) == n

    T_Gtweaked_to_B_seq = []

    T_cached = T_identity
    for i in range(n):
        if i == 0:
            T_G_to_pre = T_tool_to_obj_seq[i]
        else:
            T_G_to_obj = T_tool_to_obj_seq[i]
            T_pre_to_obj = T_tool_to_obj_seq[i-1]
            T_obj_to_pre = utils.invert_tf(T_pre_to_obj)

            T_G_to_pre = utils.convert(
                    T_G_to_obj,
                    T_obj_to_pre)

        T_tweak = T_tweak_seq[i]
        T_Gtweaked_to_G = utils.convert(T_tweak, T_G_to_pre)
        T_Gtweaked_to_B = utils.convert(T_Gtweaked_to_G, T_cached)
        T_Gtweaked_to_B_seq.append(T_Gtweaked_to_B)
        T_cached = copy.copy(T_Gtweaked_to_B)

    return T_Gtweaked_to_B_seq

def no_tweak_rule(param, n):
    T_tweak_seq = [T_identity for i in range(n)]
    return T_tweak_seq

class Reproducer:
    def __init__(self, tweak_rule = no_tweak_rule):
        self.srv = rospy.Service('get_tweak', JsonString, self._handle_tweak)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self._callback_to_tf)
        self.listener = tf.TransformListener()
        self.tweak_rule = tweak_rule
        self.T_B2I = None #hypothesized Body to Inertial (real body is denoted by Br ins.of. B)
        self.br = tf.TransformBroadcaster()

    def _handle_tweak(self, req):
        print("asked")
        print(str(req.message))
        string = str(req.message)
        dict_req = json.loads(string)

        # processing the dict
        trajectory_file = dict_req['name'] + ".traj"
        with open(trajectory_file, 'r') as f:
            data = json.load(f)

        hasParam = ('param' in dict_req)
        param = dict_req['param'] if hasParam else [0 for i in range(100)]

        hasErr = ('err' in dict_req)
        err = dict_req['err'] if hasErr else [0 for i in range(6)] # error in xyzrpy

        time.sleep(1)
        obj_frame= str(data['wrt'])


        T_G2B_seq  = data['tfs_r']
        n = len(T_G2B_seq)
        T_tweak_seq = self.tweak_rule(param, n)
        T_Gt2B_seq = tweak(T_G2B_seq, T_tweak_seq)
        
        # frame B above is distorted one with hypothesized error, thus we must compute T_Gt2Br
        # where Br denotes 'real' B
        trans_B2Br = [err[0]+eps, err[1]+eps, err[2]+eps]
        rot_B2Br = tf.transformations.quaternion_from_euler(err[3], err[4], err[5])
        T_B2Br = [trans_B2Br, rot_B2Br]
        T_Gt2Br_seq = [
                utils.convert(T_Gt2B, T_B2Br) for
                T_Gt2B in T_Gt2B_seq]

        while True:
            try:
                T_Br2I = self.listener.lookupTransform('/base_footprint', obj_frame, rospy.Time(0))
                break
            except:
                pass


        print(T_B2Br)
        print(T_Br2I)
        print(utils.convert(T_B2Br, T_Br2I))
        self.T_B2I = utils.convert(T_B2Br, T_Br2I)

        T_Gt2I_seq = [
                utils.convert(T_Gt2Br, T_Br2I) for 
                T_Gt2Br in T_Gt2Br_seq]

        data_res = {
                'T_rt_seq': T_Gt2I_seq, 
                'av_seq': data['avs']
                }
        return JsonStringResponse(message=json.dumps(data_res))

    def _callback_to_tf(self, msg): # this doesn't have to be callback
        if self.T_B2I is not None:
            trans, rot =  self.T_B2I
            self.br.sendTransform(tuple(trans), tuple(rot), rospy.Time.now(), "hypothesized_object", "base_footprint")



'''
def make_pose_msg(pose):
    trans = pose[0]
    rot = pose[1]
    point_msg = Point(x=trans[0], y=trans[1], z=trans[2])
    quaternion_msg = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
    msg = Pose(position=point_msg, orientation=quaternion_msg)
    return msg

def pub_trajectory():
    with open("./test.traj", 'r') as f:
        data = json.load(f)

    obj_frame= str(data['wrt'])

    T_rt_to_base_seq = []
    T_lt_to_base_seq = []

    time.sleep(1)
    T_B2O = listener.lookupTransform('/base_footprint', obj_frame, rospy.Time(0))

    T_G2B_seq = data['tfs_r']
    convert = lambda T_G2B: utils.convert(T_G2B, T_B2O)
    T_G2O_seq = map(convert, T_G2B_seq)
    pose_list = map(make_pose_msg, T_G2O_seq)
    pose_msg = PoseArray(poses=pose_list)

    pub = rospy.Publisher('wrt', PoseArray, queue_size=2)
    def cb(msg):
        header = copy.copy(msg.header)
        header.frame_id = '/base_footprint'
        pose_msg.header = header
        pub.publish(pose_msg)
    sub = rospy.Subscriber('/joint_states', JointState, cb)
'''

#rospy.spin()
#srv_get_tweak.shutdown()
if __name__ == '__main__':
    rospy.init_node('commander_test')
    rp = Reproducer()




