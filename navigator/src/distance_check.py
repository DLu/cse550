#!/usr/bin/python

import roslib; roslib.load_manifest('navigator')
from assignment05.path_planning import *

from navigator import *

def create_text_marker(x, y, text, resolution, id=0):
    m = Marker()
    m.header.frame_id = '/map'
    m.header.stamp = rospy.Time.now()
    m.ns = 'text'
    m.id = id
    m.type = Marker.TEXT_VIEW_FACING
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.orientation.w = 1.0
    m.scale.x = 0.7 * resolution 
    m.scale.y = 0.7 * resolution
    m.scale.z = 0.7 * resolution
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.color.a = 1.0
    m.text = text
    return m

# Parse Args
parser = argparse.ArgumentParser(description='Distance Check')
parser.add_argument('mapbag')
parser.add_argument('-c', '--cubes', action="store_true")
parser.add_argument('-t', '--text', action="store_true")

args = parser.parse_args(rospy.myargv()[1:])

# Get Data From Bag Files
the_map = get_dict( args.mapbag )['/map']

rospy.init_node('distance_check')
mpub = rospy.Publisher('/map', OccupancyGrid, latch=True)
mpub.publish(the_map)
pub = rospy.Publisher('/visualization_markers', MarkerArray)

# Callback for new goal pose
def pose_sub(msg):
    x,y,theta = to_tuple(msg.pose.position, msg.pose.orientation)
    result = to_grid2(x,y, the_map)
    if not result:
        print "INVALID"
        return
    else:
        mx, my = result
    
    # Get Distances
    ds = distance_to_points(the_map, [(mx, my)])
    
    # Create Visualization Markers
    if args.cubes or args.text:
        L = max(ds.values())
        rez = the_map.info.resolution
        
        ma = MarkerArray()
        
        cubes = Marker()
        cubes.header.stamp = rospy.Time.now()
        cubes.header.frame_id = '/map'
        cubes.ns = 'cubes'
        cubes.type = Marker.CUBE_LIST
        cubes.pose.orientation.w = 1.0
        cubes.scale.x = 1.0 * rez
        cubes.scale.y = 1.0 * rez
        cubes.scale.z = 0.1
        
        for (x1,y1), d in ds.iteritems():
            a,b = to_world2(x1, y1, the_map)

            if args.text:
                ma.markers.append( create_text_marker(a,b,str(d),rez,id=len(ma.markers)))

            if args.cubes:
                r,g,b0 = hsv_to_rgb(d/float(L*1.5), 1.0, 1.0)        
                cubes.points.append(Point(a,b,0))
                cubes.colors.append(ColorRGBA(r,g,b0,0.2))

        if args.cubes:
            ma.markers.append(cubes)
        pub.publish(ma)
    else:
        # Or just print
        print '\n'.join(map(str, sorted(ds.items())))

sub = rospy.Subscriber('/goal_pose', PoseStamped, pose_sub)
rospy.spin()
