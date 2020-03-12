#!/usr/bin/python
import utils
from tf2_msgs.msg import TFMessage
import tf
import json
import rospy
from oven.srv import *
import time
import copy

rospy.init_node('commander_test')
listener = tf.TransformListener()

T_identity = [[1e-10, 1e-10, 1e-10], [1e-10, 1e-10, 1e-10, 1.0]]

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

def debug_tweak_rule(T_tool_to_obj_seq, param):
    n = len(T_tool_to_obj_seq)
    T_Gtweaked_to_B_seq = tweak(T_tool_to_obj_seq, [T_identity for i in range(n)])
    return T_Gtweaked_to_B_seq

def no_tweak_rule(T_tool_to_obj_seq, param):
    return T_tool_to_obj_seq

def handle_tweak(req):
    print("asked")
    with open("./test.traj", 'r') as f:
        data = json.load(f)

    obj_frame= str(data['wrt'])

    time.sleep(1)
    T_B_to_I = listener.lookupTransform('/base_footprint', obj_frame, rospy.Time(0))
    T_Gtweaked_to_B_seq = debug_tweak_rule(data['tfs_r'], None)
    T_Gtweaked_to_I_seq = [
            utils.convert(T_Gt_to_B, T_B_to_I) for 
            T_Gt_to_B in T_Gtweaked_to_B_seq]

    data_res = {
            'T_rt_seq': T_Gtweaked_to_I_seq, 
            'av_seq': data['avs']
            }
    return JsonStringResponse(message=json.dumps(data_res))

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


srv_get_tweak = rospy.Service("get_tweak", JsonString, handle_tweak)
print("start")
rospy.spin()
#srv_get_tweak.shutdown()

