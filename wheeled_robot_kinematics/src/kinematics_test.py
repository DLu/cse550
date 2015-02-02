#!/usr/bin/env python

import roslib; roslib.load_manifest('assignment01')
import rospy
import sys
from geometry_msgs.msg import *
from wheeled_robot_kinematics.msg import *
from wheeled_robot_kinematics.srv import *
from math import hypot, pi

def angle_diff(a, b):
    d = a - b
    return (d + pi) % (2 * pi) - pi

def matches(pose0, pose1):
    if hypot(pose0.x - pose1.x, pose0.y - pose1.y) > .05:
        return False
    return abs( angle_diff(pose0.theta, pose1.theta)) < .05

class KinematicsTest:
    def __init__(self, name):
        self.fk_correct = 0
        self.fk_total = 0
        
        self.ik_total = 0
        self.ik_correct = 0
        self.ik_minimal = 0
    
        sname = '%s/forward'%name
        rospy.loginfo("Waiting for %s"%sname)
        rospy.wait_for_service(sname)
        self.forward = rospy.ServiceProxy(sname, DiffDriveFK)

        sname = '%s/inverse'%name
        rospy.loginfo("Waiting for %s"%sname)
        rospy.wait_for_service(sname)
        self.inverse = rospy.ServiceProxy(sname, DiffDriveIK)
        
    def forward_test(self, pose0=(0,0,0), action=(0,0,1), pose1=(0,0,0), axle=1.0, radius=1.0, speed=1.0):
        p0 = apply(Pose2D, pose0)
        p1 = apply(Pose2D, pose1)
        a = apply(DiffDriveAction, action)
        rospy.set_param('/axle_length', axle)
        rospy.set_param('/wheel_radius', radius)
        rospy.set_param('/max_speed', speed)
        resp = self.forward(p0, a)
        self.fk_total += 1
        if matches(p1, resp.end_pose):
            self.fk_correct += 1
        else:
            rospy.loginfo("Failed Forward Kinematic Test: f( %s , %s)"%(str(pose0), str(action)))
            rospy.loginfo("   Expected: %.3f %.3f %.3f"%pose1)
            rospy.loginfo("   Received: %.3f %.3f %.3f"%(resp.end_pose.x, resp.end_pose.y, resp.end_pose.theta))
            
    def inverse_test(self, pose0=(0,0,0), pose1=(0,0,0), axle=1.0, radius=1.0, speed=1.0, moves=3):
        p0 = apply(Pose2D, pose0)
        p1 = apply(Pose2D, pose1)
        rospy.set_param('/axle_length', axle)
        rospy.set_param('/wheel_radius', radius)
        rospy.set_param('/max_speed', speed)
        resp = self.inverse(p0, p1)
        self.ik_total += 1
        
        p = p0
        for a in resp.actions:
            if abs(a.left_velocity)>speed or abs(a.right_velocity)>speed:
                rospy.loginfo("Failed Inverse Kinematic Test: f( %s , %s)"%(str(pose0), str(pose1)))
                rospy.loginfo("   Action %s too fast"%((a.left_velocity, a.right_velocity, a.time)))
            resp2 = self.forward(p, a)
            p = resp2.end_pose
            
        if matches(p1, p):
            self.ik_correct += 1
        else:
            rospy.loginfo("Failed Inverse Kinematic Test: f( %s , %s)"%(str(pose0), str(pose1)))
            rospy.loginfo("   Actions: ")
            for a in resp.actions:
                rospy.loginfo("      %s"%(str((a.left_velocity, a.right_velocity, a.time))))
            rospy.loginfo("   Expected: %.3f %.3f %.3f"%pose1)
            rospy.loginfo("   Received: %.3f %.3f %.3f"%(p.x, p.y, p.theta))
            
        if len(resp.actions)==moves:
            self.ik_minimal += 1
        else:
            rospy.loginfo("Inverse Kinematic Test: f( %s , %s)"%(str(pose0), str(pose1)))
            rospy.loginfo("   Done in %d moves, expected %d."%(len(resp.actions), moves))
        
    
    def stats(self):
        rospy.loginfo("Forward Tests Correct: %d/%d"%(self.fk_correct, self.fk_total))
        rospy.loginfo("Inverse Tests Correct: %d/%d"%(self.ik_correct, self.ik_total))
        rospy.loginfo("Inverse Tests in Minimum Moves: %d/%d"%(self.ik_minimal, self.ik_total))
        
if __name__ == "__main__":
    if len(sys.argv)<=1:
        print "Required argument: name of the node!"
    rospy.init_node('kinematic_tester', anonymous=True)
    name = sys.argv[1]
    kt = KinematicsTest(name)
    kt.forward_test()
    kt.forward_test(action=(3,3,1),pose1=(3,0,0))
    kt.forward_test(action=(3,-3,1),pose1=(0,0,-6))
    kt.forward_test(action=(-3,3,1),pose1=(0,0,6))
    C1 = 2.104
    C2 = 1.149
    kt.forward_test(action=( 3, 2,1),pose1=( C1,-C2,-1))
    kt.forward_test(action=( 2, 3,1),pose1=( C1, C2, 1))
    kt.forward_test(action=(-3,-2,1),pose1=(-C1,-C2, 1))
    kt.forward_test(action=(-2,-3,1),pose1=(-C1, C2,-1))
    
    kt.forward_test(action=(3,3,1),pose1=(3,0,0), axle=0.5)
    kt.forward_test(action=(3,3,1),pose1=(1.5,0,0), radius=0.5)

    C1 = 1.137
    C2 = 1.770
    kt.forward_test(action=( 3, 2,1),pose1=( C1,-C2,-2), axle=0.5)
    kt.forward_test(action=( 2, 3,1),pose1=( C1, C2,2), axle=0.5)
    kt.forward_test(action=(-3,-2,1),pose1=(-C1,-C2,2), axle=0.5)
    kt.forward_test(action=(-2,-3,1),pose1=(-C1, C2,-2), axle=0.5)
    
    kt.forward_test(pose0=(1,5,0),action=(3,3,10),pose1=(31,5,0))
    kt.forward_test(pose0=(-1,-1,pi),action=(3,3,1),pose1=(-4,-1,pi))
    kt.forward_test(pose0=(-1,-1,pi/2),action=(3,3,1),pose1=(-1,2,pi/2))
    kt.forward_test(pose0=(-5,0,pi/3),action=(3,3,5),pose1=(2.5,12.99,pi/3))
    
    kt.forward_test(pose0=(3,3,-pi/5),action=(1,5,8.3),pose1=(4.127, 3.766, 1.156))
    kt.forward_test(pose0=(3,3,-pi/5),action=(1,5,8.3),pose1=(3.080,3.256,-3.040), axle=0.22, radius=0.9)
        
    kt.inverse_test(moves=0)
    kt.inverse_test(pose1=(3,0,   0), moves=1)
    kt.inverse_test(pose1=(0,0,  pi), moves=1)
    kt.inverse_test(pose1=(0,0, -pi), moves=1)
    kt.inverse_test(pose1=(0,0,pi/3), moves=1)
    
    C1 = 2.104
    C2 = 1.149
    kt.inverse_test(pose1=( C1,-C2,-1), moves=1)
    kt.inverse_test(pose1=( C1, C2, 1), moves=1)
    kt.inverse_test(pose1=(-C1,-C2, 1), moves=1)
    kt.inverse_test(pose1=(-C1, C2,-1), moves=1)
    
    kt.inverse_test(pose0=(1,1,0), pose1=(2,2,0), moves=2)
    
    kt.inverse_test(pose1=(0,0,pi), axle=0.5, moves=1)
    kt.inverse_test(pose1=(1.5,0,0), radius=0.5, moves=1)
    
    kt.stats()
