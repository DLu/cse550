import roslib; roslib.load_manifest('localization')
from localization import *
from localization.bag import get_dict
from assignment02.geometry import *
from assignment04.laser import *
from math import pi
import tf
from tf.transformations import euler_from_quaternion
import argparse

import rospy
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *

# Parse Args
parser = argparse.ArgumentParser(description='Pose Scorer')
parser.add_argument('mapbag')
parser.add_argument('databag')

args = parser.parse_args()

# Get Data From Bag Files
the_map = get_dict( args.mapbag )['/map']
test_files = get_dict( args.databag )
scan = test_files['/base_scan']
truth = test_files['/base_pose_ground_truth']

pose = truth.pose.pose
true_pos = pose.position.x, pose.position.y, euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[2]
print "True Position:", true_pos

scan2 = LaserScan()
scan2.header = scan.header
scan2.angle_min = scan.angle_min
scan2.angle_max = scan.angle_max
scan2.angle_increment = scan.angle_increment
scan2.range_max = scan.range_max

rospy.init_node('query')
mpub = rospy.Publisher('/map', OccupancyGrid, latch=True)
mpub.publish(the_map)
pub = rospy.Publisher('/base_scan', LaserScan)
br = tf.TransformBroadcaster()

def pose_sub(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    theta = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    print x,y,theta
    result = to_grid(x,y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
    if not result:
        print "INVALID"
        return
    else:
        mx, my = result
    
    ex_scan = expected_scan(mx, my, theta, scan.angle_min, scan.angle_increment, len(scan.ranges), scan.range_max, the_map)
    scan2.ranges = ex_scan
    
    publish_update(pub, scan2, br, (x,y,theta))
    
    score = scan_similarity(scan.ranges, ex_scan, scan.range_max)
    print score
    

sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, pose_sub)
rospy.spin()
