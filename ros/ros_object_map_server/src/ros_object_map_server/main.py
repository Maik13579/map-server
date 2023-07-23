#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import OccupancyGrid

from tf2_geometry_msgs import do_transform_pose

from object_map_server import load_objects, load_frames, Frame, save_objects, save_frames, \
object_as_sdf, cube_plane_intersection, cylinder_plane_intersection, intersection_to_grid_map
from conversion import objects2markerarray, pose2rospose, compose_poses, object2markerarray, create_occupancy_grid
from interactive_object_server import InteractiveObjectServer

from ros_object_map_server_interfaces.srv import Save, Load, SaveResponse, LoadResponse
from std_srvs.srv import Trigger, TriggerResponse

from gazebo_msgs.srv import SpawnModel

OBJECT_TOPIC = "/object_map_server/objects"
MAP_TOPIC = "/object_map_server/map"
RATE = 5

class ObjectMapServer():
    def __init__(self, object_path: str, frame_path: str, root_frame_name: str='map'):
        rospy.init_node("object_map_server", anonymous=True)

        #create service
        self.save_service = rospy.Service('object_map_server/save', Save, self.service_save)
        self.load_service = rospy.Service('object_map_server/load', Load, self.service_load)
        self.spawn_gazebo_service = rospy.Service('object_map_server/spawn_gazebo', Trigger, self.service_spawn_gazebo)
        self.generate_occupancy_grid_service = rospy.Service('object_map_server/generate_occupancy_grid', Trigger, self.service_generate_occupancy_grid)
        
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
        self.pub_grid = rospy.Publisher(MAP_TOPIC, OccupancyGrid, queue_size=10)

        # Tf broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Tf listener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        self.rate = rospy.Rate(RATE)
        self.occupancy_grid = None

        while not rospy.is_shutdown():
            self.publish_objects()
            self.publish_tf_recursiv(self.root_frame)
            self.server.main_loop()
            
            if self.occupancy_grid is not None:
                self.occupancy_grid.header.stamp = rospy.Time.now()
                self.pub_grid.publish(self.occupancy_grid)
            self.rate.sleep()

    def service_generate_occupancy_grid(self, request):
        response = TriggerResponse()
        rospy.loginfo('Generating occupancy grid...')
        try:
            self.occupancy_grid = self.generate_occupancy_grid(self.root_frame_name, 10.0, 10.0, 0.04)
        except Exception as e:
            response.success = False
            response.message = str(e)
            return response
        response.success = True
        response.message = "Generated occupancy grid in frame " + self.root_frame_name + " with size 10x10m and resolution 0.04m, published at " + MAP_TOPIC
        return response

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

    def generate_occupancy_grid(self, frame, width, height, resolution):
        '''
        generate occupancy grid
        '''
        all_intersections = []
        for obj in self.objects:
            #transform object to [frame]

            #get transform to target frame
            transform = self.tf_buffer.lookup_transform(frame,
                                       obj.name,
                                       rospy.Time.now(),
                                       rospy.Duration(10.0))
            #convert to marker array
            obj_markers = object2markerarray(obj)

            #transform markers to frame
            transformed_markers = []
            for marker in obj_markers.markers:
                posestamped = PoseStamped()
                posestamped.header.stamp = rospy.Time.now()
                posestamped.header.frame_id = obj.name
                posestamped.pose = marker.pose
                marker.pose = do_transform_pose(posestamped, transform)
                transformed_markers.append(marker)

            #compute intersections
            for marker in transformed_markers:
                if marker.type == 1: #cube
                    if marker.text == 'bb': #skip bounding boxes
                        continue
                    intersection = cube_plane_intersection(
                        (marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z),
                        (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w),
                        (marker.scale.x, marker.scale.y, marker.scale.z),
                        (0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.1) #height of laser scanner from PAL tiago rougly is 10 cm
                    )
                    if len(intersection) > 0:
                        all_intersections.append(intersection)

                elif marker.type == 3: #cylinder
                    intersection = cylinder_plane_intersection(
                        (marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z),
                        (marker.pose.pose.orientation.x, marker.pose.pose.orientation.y, marker.pose.pose.orientation.z, marker.pose.pose.orientation.w),
                        marker.scale.z,
                        marker.scale.x/2,
                        (0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.1) #height of laser scanner from PAL tiago rougly is 10 cm
                    )
                    if len(intersection) > 0:
                        all_intersections.append(intersection)
        #convert to grid map
        image, w, h = intersection_to_grid_map(all_intersections, (width, height), resolution)
        occupancy_grid = create_occupancy_grid(resolution, frame, image, w, h)
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.info.map_load_time = rospy.Time.now()
        return occupancy_grid


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 2:
        object_path = sys.argv[2]
        frame_path = sys.argv[1]
    else:
        print("Usage: ros2 run ros_object_map_server main.py <frame_path> <object_path>")
        exit()

    ObjectMapServer(object_path, frame_path)

