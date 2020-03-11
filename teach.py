#!/usr/bin/python

import rospy
import numpy as np
import math
import tf
from sensor_msgs.msg import JointState
import copy
import utils
import json

class Chunker:
    def __init__(self, object_frame):
        self.angle_vector = None
        self.object_frame = object_frame
        self.data = {'tfs_r': [], 'tfs_l': [], 'avs': [], 'n': 0}

        self.listener = tf.TransformListener()
        self.sub = rospy.Subscriber('/joint_states', JointState, self._callback_joint_states)

    def _callback_joint_states(self, msg):
        D = {}
        for name, position in zip(msg.name, msg.position):
            D[name] = position

        angle_vector = []
        for name in utils.pr2_joint_names():
            if name=='torso_lift_joint':
                angle_vector.append(D[name] * 1000)
            else:
                angle_vector.append(D[name] * 180/math.pi)
        self.angle_vector = angle_vector

    def _store_single_waypoint(self):

        print("hit any key: if q, then the teaching process will terminate")
        string = raw_input()
        if string=='q':
            return False
        
        while True:
            try:
                tf_r, tf_l = [self.listener.lookupTransform(
                    self.object_frame, frame, rospy.Time(0)) 
                    for frame in ['r_gripper_tool_frame', 'l_gripper_tool_frame']]
                self.data['tfs_r'].append(tf_r)
                self.data['tfs_l'].append(tf_l)
                self.data['avs'].append(self.angle_vector)
                self.data['n'] += 1
                return True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

    def run(self):
        while self._store_single_waypoint():
            pass
        self.sub.unregister()

if __name__=='__main__':
    test = True
    rospy.init_node('teach')
    object_frame = '/base_link' if test else 'object'
    C = Chunker(object_frame)
    C.run()

    data = C.data
    data['wrt'] = object_frame

    print("filename?")
    filename = raw_input() + ".traj"
    with open(filename, 'wb') as f:
        json.dump(C.data, f, ensure_ascii=False)







