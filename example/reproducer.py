#!/usr/bin/python
import rospy
import tf
import trajectory as traj
rospy.init_node('commander_test')

def full_tweak_rule(param, n):
    param[0] = 0
    T_id = traj.T_identity
    T_tweaks = [T_id]

    Q_rolled = lambda angle: tf.transformations.quaternion_from_euler(0, angle, 0)

    # tweak except initial waypoint
    for i in range(n-1):
        trans = [param[3*i+0], param[3*i+1], traj.eps]
        rot = Q_rolled(param[3*i+2])
        T_tweak = [trans, rot]
        T_tweaks.append(T_tweak)
    print("fufaefjaoeifjoaie")
    return T_tweaks

rp = traj.Reproducer()
rospy.spin()
