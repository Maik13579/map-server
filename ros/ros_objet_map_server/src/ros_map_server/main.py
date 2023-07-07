#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray

from object_map_server import load_objects, load_frames, Frame
from conversion import objects2markerarray, pose2rospose

OBJECT_TOPIC = "/object_map_server/objects"
RATE = 1.0

class ObjectMapServer():
    def __init__(self, object_path: str, frame_path: str):
        rospy.init_node("object_map_server", anonymous=True)
        
        rospy.loginfo("Loading objects")
        self.objects = load_objects(object_path)
        
        rospy.loginfo("Loading frames")
        self.root_frame = load_frames(frame_path)
        
        # Publisher
        self.pub = rospy.Publisher(OBJECT_TOPIC, MarkerArray, queue_size=10)

        # Tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.publish_objects()
            self.publish_tf_recursiv(self.root_frame)
            self.rate.sleep()

    def publish_objects(self):
        markers = objects2markerarray(self.objects)
        self.pub.publish(markers)

    def publish_tf(self, frame_id, child_frame_id, pose):
        '''
        publish tf
        '''
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        self.br.sendTransform(t)

    def publish_tf_recursiv(self, frame):
        '''
        publish tf recursivly
        '''
        #if frame has parent, publish tf
        if isinstance(frame.parent, Frame):
            self.publish_tf(frame.parent.name, frame.name, pose2rospose(frame.pose))

        #recursivly call this function for all children
        for child in frame.children:
            self.publish_tf_recursiv(child)



if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Object Map Server')
    parser.add_argument('--object_path', type=str, help='path to objects')
    parser.add_argument('--frame_path', type=str, help='path to frames')
    args = parser.parse_args()

    ObjectMapServer(args.object_path, args.frame_path)

