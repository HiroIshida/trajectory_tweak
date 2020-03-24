#!/usr/bin/python
import rospy
import trajectory as traj
rospy.init_node('commander_test')
rp = traj.Reproducer()
rospy.spin()
