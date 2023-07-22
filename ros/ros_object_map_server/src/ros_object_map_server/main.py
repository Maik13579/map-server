#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose
from visualization_msgs.msg import MarkerArray

from object_map_server import load_objects, load_frames, Frame, save_objects, save_frames, object_as_sdf
from conversion import objects2markerarray, pose2rospose, compose_poses
from interactive_object_server import InteractiveObjectServer

from ros_object_map_server_interfaces.srv import Save, Load, SaveResponse, LoadResponse
from std_srvs.srv import Trigger, TriggerResponse

from gazebo_msgs.srv import SpawnModel

OBJECT_TOPIC = "/object_map_server/objects"
RATE = 5

class ObjectMapServer():
    def __init__(self, object_path: str, frame_path: str, root_frame_name: str='map'):
        rospy.init_node("object_map_server", anonymous=True)

        #create service
        self.save_service = rospy.Service('object_map_server/save', Save, self.service_save)
        self.load_service = rospy.Service('object_map_server/load', Load, self.service_load)
        self.spawn_gazebo_service = rospy.Service('object_map_server/spawn_gazebo', Trigger, self.service_spawn_gazebo)
        
        rospy.loginfo("Loading objects")
        self.object_path = object_path
        self.objects = load_objects(self.object_path)
        
        rospy.loginfo("Loading frames")
        self.frame_path = frame_path
        self.root_frame_name = root_frame_name
        self.root_frame = load_frames(self.frame_path, self.root_frame_name)

        #create interactive marker server
        self.server = InteractiveObjectServer("interactive_object_map_server", self.objects, self.root_frame)
        
        # Publisher
        self.pub = rospy.Publisher(OBJECT_TOPIC, MarkerArray, queue_size=10)

        # Tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            self.publish_objects()
            self.publish_tf_recursiv(self.root_frame)
            self.server.main_loop()
            self.rate.sleep()

    def service_spawn_gazebo(self, request):
        response = TriggerResponse()
        rospy.loginfo('Spawning objects in gazebo...')
        try:
            #create service client
            cli = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            self.spawn_gazebo_recusriv(self.root_frame, Pose(), cli)
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response
        response.success = True
        response.message = "Spawned objects in gazebo"
        return response
    
    def spawn_gazebo_recusriv(self, frame, parent_pose, cli):
        frame_pose = compose_poses(parent_pose, pose2rospose(frame.pose))

        #find object with same name as frame
        for obj in self.objects:
            if obj.name == frame.name:
                #spawn object
                xml = object_as_sdf(obj)
                cli(obj.name, xml, '', frame_pose, '')
                break
        for child in frame.children:
            self.spawn_gazebo_recusriv(child, frame_pose, cli)

    def service_save(self, request):
        response = SaveResponse()
        rospy.loginfo('Saving...')
        try:
            frame_path = request.frame_path if request.frame_path!='' else self.frame_path
            object_path = request.object_path if request.object_path!='' else self.object_path
            save_frames(self.root_frame, frame_path)
            save_objects(self.objects, object_path)
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response
        #remember paths
        self.frame_path = frame_path
        self.object_path = object_path
        response.success = True
        response.message = "Saved objects at " + self.object_path + " and frames at " + self.frame_path
        return response
    
    def service_load(self, request):
        response = LoadResponse()
        rospy.loginfo('Loading...')
        try:
            frame_path = request.frame_path if request.frame_path!='' else self.frame_path
            root_frame_name = request.root_frame_name if request.root_frame_name!='' else self.root_frame_name
            object_path = request.object_path if request.object_path!='' else self.object_path
            self.objects = load_objects(object_path)
            self.root_frame = load_frames(frame_path, self.root_frame_name)
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response
        
        #remember paths
        self.object_path = object_path
        self.frame_path = frame_path
        self.root_frame_name = root_frame_name
   
        
        #update interactive marker server
        self.server.objects = self.objects
        self.server.root_frame = self.root_frame
        self.server.interactive_objects = self.server.load_objects()

        response.success = True
        response.message = "Loaded objects from " + self.object_path + " and frames from " + self.frame_path
        return response


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

