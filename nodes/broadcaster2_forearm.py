#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped


def handle_arm_pose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "forearm"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.x
    t.transform.rotation.y = msg.y
    t.transform.rotation.z = msg.z
    t.transform.rotation.w = msg.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('broadcast_forearm')
    rospy.Subscriber('forearm_quaternion',
                     Quaternion,
                     handle_arm_pose)
    rospy.spin()