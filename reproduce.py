import utils
from tf2_msgs.msg import TFMessage
import tf
import json
import rospy
from oven.srv import *
import time

rospy.init_node('commander_test')
listener = tf.TransformListener()

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

def handle_tweak(req):
    data_res = {
            'T_rt_seq': T_rt_to_base_seq, 
            'T_lt_seq': T_lt_to_base_seq,
            'av_seq': data['avs']
            }
    return JsonStringResponse(message=json.dumps(data_res))

srv_get_tweak = rospy.Service("get_tweak", JsonString, handle_tweak)
srv_get_tweak.shutdown()

