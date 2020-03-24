#!/usr/bin/python
import rospy
import tf

rospy.init_node('dumy_object_pub')
bc = tf.TransformBroadcaster()
rot = tf.transformations.quaternion_from_euler(0, 0.0, 3.1415/4)
trans = [0.7, -0.05, 0.723]

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    bc.sendTransform(trans, rot, rospy.Time.now(), "object", "base_link")
    rate.sleep()
