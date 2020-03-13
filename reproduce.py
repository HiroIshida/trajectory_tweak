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
import tweak

rospy.init_node('commander_test')
listener = tf.TransformListener()


def handle_tweak(req):
    print("asked")
    with open("./test.traj", 'r') as f:
        data = json.load(f)

    obj_frame= str(data['wrt'])

    time.sleep(1)
    T_B_to_I = listener.lookupTransform('/base_footprint', obj_frame, rospy.Time(0))
    T_Gtweaked_to_B_seq = tweak.simple_tweak_rule(data['tfs_r'], 0.0)
    T_Gtweaked_to_I_seq = [
            utils.convert(T_Gt_to_B, T_B_to_I) for 
            T_Gt_to_B in T_Gtweaked_to_B_seq]

    data_res = {
            'T_rt_seq': T_Gtweaked_to_I_seq, 
            'av_seq': data['avs']
            }
    return JsonStringResponse(message=json.dumps(data_res))

def pub_trajecgtory(T_G2B_seq, T_B2O):
    convert = lambda T_G2B: utils.convert(T_G2B, T_B2O)
    T_G2O_seq = map(convert, T_G2B_seq)
    msg = PoseArray()



def __handle_tweak(req):
    def read_json():
        with open("./test.traj", 'r') as f:
            data = json.load(f)

        obj_frame= str(data['wrt'])

        T_rt_to_base_seq = []
        T_lt_to_base_seq = []

        time.sleep(1)
        T_obj_to_base = listener.lookupTransform('/base_footprint', obj_frame, rospy.Time(0))

        for i in range(data['n']):
            T_rt_to_obj = data['tfs_r'][i] # r tool
            T_lt_to_obj = data['tfs_l'][i] # l tool
            T_rt_to_base = utils.convert(T_rt_to_obj, T_obj_to_base)
            T_lt_to_base = utils.convert(T_lt_to_obj, T_obj_to_base)

            T_rt_to_base_seq.append(T_rt_to_base)
            T_lt_to_base_seq.append(T_lt_to_base)

        data_res = {
                'T_rt_seq': T_rt_to_base_seq, 
                'T_lt_seq': T_lt_to_base_seq,
                'av_seq': data['avs']
                }
        return data_res
    data_res = read_json()
    return JsonStringResponse(message=json.dumps(data_res))

def make_pose_msg(pose):
    trans = pose[0]
    rot = pose[1]
    point_msg = Point(x=trans[0], y=trans[1], z=trans[2])
    quaternion_msg = Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])
    msg = Pose(position=point_msg, orientation=quaternion_msg)
    return msg

#rospy.spin()
#srv_get_tweak.shutdown()
if __name__ == '__main__':
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
    rospy.spin()



