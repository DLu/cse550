import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def to_pose(x,y,theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    quat = quaternion_from_euler(0,0,theta)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    return pose
    
def to_tuple(position, orientation):
    x = position.x
    y = position.y 
    theta = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]
    return x,y,theta

def publish_update(pub, scan, br, (x,y,theta)):
    r = rospy.Rate(10)
    for i in range(10):
        scan.header.stamp = rospy.Time.now()
        pub.publish(scan)
        br.sendTransform((x, y, 0),
                         quaternion_from_euler(0, 0, theta),
                         rospy.Time.now(),
                         '/base_laser_link',
                         "/map")
        r.sleep()   
