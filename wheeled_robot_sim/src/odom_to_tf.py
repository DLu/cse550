#!/usr/bin/python

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomConverter:
    def __init__(self):
        rospy.init_node('odom_to_tf')
        self.sub = rospy.Subscriber('odom', Odometry, self.on_odom)
        self.br = tf.TransformBroadcaster()

    def on_odom(self, msg):
        pose = msg.pose.pose
        pos = (pose.position.x, pose.position.y, pose.position.z)
        quat = (pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w)
        self.br.sendTransform(pos, quat, msg.header.stamp,
                     '/base_link', msg.header.frame_id)

OdomConverter()
rospy.spin()

