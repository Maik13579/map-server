
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray

from object_map_server import load_objects, load_frames, Frame
from ros_object_map_server.conversion import objects2markerarray, pose2rospose
from ros_object_map_server.interactive_object_server import InteractiveObjectServer

OBJECT_TOPIC = "/object_map_server/objects"
RATE = 5.0

class ObjectMapServer(Node):
    def __init__(self, object_path: str, frame_path: str):
        super().__init__('object_map_server')
        
        self.get_logger().info('Loading objects')
        self.objects = load_objects(object_path)
        
        self.get_logger().info('Loading frames')
        self.root_frame = load_frames(frame_path)
        
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