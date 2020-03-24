#!/usr/bin/python
import utils
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
import tf
import json
import rospy
from oven.srv import *
import time
import copy
#import tweak


def no_tweak_rule(T_G2B_seq, param):
    return T_G2B_seq

class Reproducer:
    def __init__(self, tweak_rule = no_tweak_rule):
        self.srv = rospy.Service('get_tweak', JsonString, self._handle_tweak)
        self.listener = tf.TransformListener()
        self.tweak_rule = no_tweak_rule

    def _handle_tweak(self, req):
        print("asked")
        print(req.message)
        dict_req = json.loads(str(req.message))
        print(dict_req['param'])

        trajectory_file = dict_req['name'] + ".traj"
        with open(trajectory_file, 'r') as f:
            data = json.load(f)

        time.sleep(1)
        obj_frame= str(data['wrt'])
        T_B_to_I = self.listener.lookupTransform('/base_footprint', obj_frame, rospy.Time(0))

        try:
            print("tweak!")
            param = dict_req['param'] 
        except KeyError:
            print("no tweak...")
            param = [0 for i in range(100)]

        #T_Gtweaked_to_B_seq = tweak.full_tweak_rule(data['tfs_r'], param)
        T_Gtweaked_to_B_seq = self.tweak_rule(data['tfs_r'], param)
        T_Gtweaked_to_I_seq = [
                utils.convert(T_Gt_to_B, T_B_to_I) for 
                T_Gt_to_B in T_Gtweaked_to_B_seq]

        data_res = {
                'T_rt_seq': T_Gtweaked_to_I_seq, 
                'av_seq': data['avs']
                }
        return JsonStringResponse(message=json.dumps(data_res))


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
    rospy.spin()



