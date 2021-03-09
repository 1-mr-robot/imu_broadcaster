#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv
from std_msgs.msg import Float64MultiArray
from tf.transformations import euler_from_quaternion

pi=3.14159265359

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    pub_angle = rospy.Publisher('flexion_angle', Float64MultiArray, queue_size=1)
    angle = Float64MultiArray()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('trunk', 'forearm', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        orientation_list = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        angles = [0]*3
        # roll pitch yaw
        (angles[0], angles[1], angles[2]) = euler_from_quaternion (orientation_list)
        angle.data = []
        for i in range(0,3):
            angle.data.append(angles[i]*180/pi)

        pub_angle.publish(angle)

        rate.sleep()