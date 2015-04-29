#!/usr/bin/env python  
import roslib; roslib.load_manifest('navigator')
import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('static_tf')
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     '/odom', '/map')
        rate.sleep()

    rospy.spin()
