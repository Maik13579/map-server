
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray

from object_map_server import load_objects, load_frames, Frame, save_objects, save_frames
from ros_object_map_server.conversion import objects2markerarray, pose2rospose
from ros_object_map_server.interactive_object_server import InteractiveObjectServer

from ros_object_map_server_interfaces.srv import Save, Load

OBJECT_TOPIC = "/object_map_server/objects"
RATE = 5.0

class ObjectMapServer(Node):
    def __init__(self, object_path: str, frame_path: str, root_frame_name: str='map'):
        super().__init__('object_map_server')

        #Create service
        self.save_service = self.create_service(Save, 'object_map_server/save', self.service_save)
        self.load_service = self.create_service(Load, 'object_map_server/load', self.service_load)
        
        self.get_logger().info('Loading objects')
        self.object_path = object_path
        self.objects = load_objects(self.object_path)
        
        self.get_logger().info('Loading frames')
        self.frame_path = frame_path
        self.root_frame_name = root_frame_name
        self.root_frame = load_frames(self.frame_path, self.root_frame_name)
        
        #create interactive marker server
        self.server = InteractiveObjectServer(self, "interactive_object_map_server", self.objects, self.root_frame)

        # Publisher
        qos_profile = QoSProfile(depth=10)
        self.pub = self.create_publisher(MarkerArray, OBJECT_TOPIC, qos_profile)

        # Tf broadcaster
        self.br = TransformBroadcaster(self)

        timer_period = 1.0 / RATE  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.publish_objects()
        self.publish_tf_recursiv(self.root_frame)
        self.server.main_loop()

    def service_save(self, request, response):
        self.get_logger().info('Saving...')
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
    
    def service_load(self, request, response):
        self.get_logger().info('Loading...')
        try:
            frame_path = request.frame_path if request.frame_path!='' else self.frame_path
            root_frame_name = request.root_frame_name if request.root_frame_name!='' else self.root_frame_name
            object_path = request.object_path if request.object_path!='' else self.object_path
            self.objects = load_objects(object_path)
            self.root_frame = load_frames(frame_path, root_frame_name)
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
        t.header.stamp = self.get_clock().now().to_msg()
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


def main(args=None):
    import sys
    if len(sys.argv) > 2:
        object_path = sys.argv[2]
        frame_path = sys.argv[1]
    else:
        print("Usage: ros2 run ros_object_map_server main.py <frame_path> <object_path>")
        exit()

    rclpy.init(args=args)
    try:
        node = ObjectMapServer(object_path, frame_path)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()