#!/usr/bin/python

import roslib; roslib.load_manifest('wheeled_robot_sim')
import rospy
import argparse
from std_msgs.msg import Float64
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from wheeled_robot_kinematics.msg import *
from wheeled_robot_kinematics.srv import *
import tf

x = 0
y = 0
theta = 0

def odom_callback(msg):
    global x,y,theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    theta = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]



LEFT_S = '/diffdrive/left_velocity_controller/command'
RIGHT_S = '/diffdrive/right_velocity_controller/command'

parser = argparse.ArgumentParser(description='Simulation Driver')
parser.add_argument('-x',      default=0.0, type=float)
parser.add_argument('-y',      default=0.0, type=float)
parser.add_argument('-theta',  default=0.0, type=float)
parser.add_argument('name', type=str)


args = parser.parse_args()
name = args.name

rospy.init_node('driver')
lp = rospy.Publisher(LEFT_S,  Float64)
rp = rospy.Publisher(RIGHT_S, Float64)

sub = rospy.Subscriber('/odom', Odometry, odom_callback)

sname = '%s/inverse'%name
rospy.loginfo("Waiting for %s"%sname)
rospy.wait_for_service(sname)
inverse = rospy.ServiceProxy(sname, DiffDriveIK)

rospy.sleep(.1)

p0 = Pose2D(x,y,theta)
p1 = Pose2D(args.x, args.y, args.theta)
resp = inverse(p0, p1)

r = rospy.Rate(10)

for action in resp.actions:
    t = rospy.Time.now()

    while rospy.Time.now()-t < rospy.Duration(action.time):
        lp.publish( action.left_velocity )
        rp.publish( action.right_velocity )
        r.sleep()

for i in range(10):
    lp.publish(0)
    rp.publish(0)
    r.sleep()
    
rospy.loginfo("Final position: %.2f %.2f %.2f"%(x, y, theta))
