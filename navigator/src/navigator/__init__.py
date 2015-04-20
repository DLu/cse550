from localization import *
from localization.bag import get_dict
from assignment02.geometry import *
import tf
import argparse

import rospy
from sensor_msgs.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA

def to_grid2(x, y, the_map):
    return to_grid(x,y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)
    
def to_world2(x, y, the_map):
    return to_world(x, y, the_map.info.origin.position.x, the_map.info.origin.position.y, the_map.info.width, the_map.info.height, the_map.info.resolution)

def hsv_to_rgb(h, s, v):
    if s == 0.0: return [v, v, v]
    i = int(h*6.)
    f = (h*6.)-i; p,q,t = v*(1.-s), v*(1.-s*f), v*(1.-s*(1.-f)); i%=6
    if i == 0: return [v, t, p]
    if i == 1: return [q, v, p]
    if i == 2: return [p, v, t]
    if i == 3: return [p, q, v]
    if i == 4: return [t, p, v]
    if i == 5: return [v, p, q]
    

def to_path(coords, the_map):
    path = Path()
    path.header.frame_id = '/map'
    path.header.stamp = rospy.Time.now()
    for x,y in coords:
        a,b = to_world2(x, y, the_map)
        p = PoseStamped()
        p.header.frame_id = '/map'
        p.header.stamp = rospy.Time.now()
        p.pose = to_pose(a,b,0)
        path.poses.append(p)
    return path


