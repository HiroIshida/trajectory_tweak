import copy
import numpy as np
import tf


def pr2_joint_names():
    pr2_joint_name_lst = [
            "torso_lift_joint",
            "l_shoulder_pan_joint",
            "l_shoulder_lift_joint",
            "l_upper_arm_roll_joint", 
            "l_elbow_flex_joint",
            "l_forearm_roll_joint",
            "l_wrist_flex_joint",
            "l_wrist_roll_joint",
            "r_shoulder_pan_joint",
            "r_shoulder_lift_joint",
            "r_upper_arm_roll_joint",
            "r_elbow_flex_joint",
            "r_forearm_roll_joint",
            "r_wrist_flex_joint",
            "r_wrist_roll_joint",
            "head_pan_joint",
            "head_tilt_joint"]
    return pr2_joint_name_lst

def qv_mult(q1, v1_):
    length = np.linalg.norm(v1_)
    v1 = v1_/length
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    v_converted = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]
    return v_converted * length


def convert(tf_12, tf_23):
    tran_12, rot_12 = [np.array(e) for e in tf_12]
    tran_23, rot_23 = [np.array(e) for e in tf_23]

    rot_13 = tf.transformations.quaternion_multiply(rot_12, rot_23)
    tran_13 = tran_12 + qv_mult(rot_12, tran_23)
    return list(tran_13), list(rot_13)

def normalize(vec):
    return vec/np.linalg.norm(vec)

def invert_tf(tf_inp):
    trans, rot = tf_inp
    rot_ = copy.copy(rot)
    for i in range(3):
        rot_[i] *= -1
    rot_ = normalize(rot_)
    trans_ = qv_mult(rot_, [-e for e in trans])

    if np.isnan(trans_[0]):
        trans_ = [0, 0, 0]
    return [trans_, rot_]

if __name__=='__main__':
    trans = [0.0513, 0, 0.025]
    rot = [-0.5, 0.4999999999, -0.5, 0.5]
    tf0 = [trans, rot]
    tf1 = invert_tf(tf0)
    tf2 = convert(tf0, tf1)


