import rosbag

def get_dict(filename):
    bag = rosbag.Bag(filename)
    X = {}
    for topic, msg, t in bag.read_messages():
        X[topic] = msg
    return X    
