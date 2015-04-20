#!/usr/bin/python

import roslib; roslib.load_manifest('navigator')
from assignment05.path_planning import *

from navigator import *

# Parse Args
parser = argparse.ArgumentParser(description='Path Plan')
parser.add_argument('mapbag')

args = parser.parse_args(rospy.myargv()[1:])

# Get Data From Bag Files
the_map = get_dict( args.mapbag )['/map']

rospy.init_node('path_plan')
mpub = rospy.Publisher('/map', OccupancyGrid, latch=True)
mpub.publish(the_map)
ppub = rospy.Publisher('/start_pose', PoseStamped, latch=True)
path_pub = rospy.Publisher('/path', Path)

start_pose = None
goal_pose = None

def plan():
    if start_pose is None or goal_pose is None:
        return
    result = to_grid2(start_pose[0],start_pose[1], the_map)
    if not result:
        print "INVALID START POSE"
        return
    else:
        sx, sy = result
    result = to_grid2(goal_pose[0],goal_pose[1], the_map)
    if not result:
        print "INVALID GOAL POSE"
        return
    else:
        gx, gy = result
        
    X = plan_path(sx, sy, gx, gy, the_map)
    if X:
        path_pub.publish(to_path(X, the_map))
    else:
        print "NO PATH"

def goal_sub(msg):
    global goal_pose
    goal_pose = to_tuple(msg.pose.position, msg.pose.orientation)
    plan()
            
def start_sub(msg):
    global start_pose
    start_pose = to_tuple(msg.pose.pose.position, msg.pose.pose.orientation)
    ps = PoseStamped()
    ps.header = msg.header
    ps.pose = apply(to_pose, start_pose)
    ppub.publish(ps)
    plan()

sub = rospy.Subscriber('/goal_pose', PoseStamped, goal_sub)
sub2 = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, start_sub)
rospy.spin()
