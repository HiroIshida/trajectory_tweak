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
    def __init__(self, object_frame, gripper_frame):
        self.angle_vector = None
        self.object_frame = object_frame
        self.gripper_frame = gripper_frame
        self.data = {'tfs': [], 'avs': []}

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
                tf_relative = self.listener.lookupTransform(
                        self.object_frame, self.gripper_frame, rospy.Time(0))
                self.data['tfs'].append(tf_relative)
                self.data['avs'].append(self.angle_vector)
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
    object_frame = '/l_gripper_tool_frame' if test else 'object'
    gripper_frame = '/r_gripper_tool_frame'
    C = Chunker(object_frame, gripper_frame)
    C.run()

    print("filename?")
    filename = raw_input() + ".traj"
    with open(filename, 'wb') as f:
        json.dump(C.data, f)







