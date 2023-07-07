#python functions to convert stuff to ros

import tf.transformations as tft
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose as RosPose
from object_map_server.interfaces import Pose, Geometry


#values from http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
MARKER_TYPE = {"CUBE":1, "SPHERE":2, "CYLINDER":3, "MESH_RESOURCE": 10}

def geometry2marker(geometry: Geometry, namespace: str, frame_id: str, id: int):
    '''
    convert object_map_server.interfaces.Geometry to visualization_msgs.Marker
    '''
    marker = Marker()
    marker.ns = namespace
    marker.id = id
    marker.type = MARKER_TYPE[geometry.type] #convert type
    marker.mesh_resource = geometry.mesh_resource if geometry.type == "MESH_RESOURCE" else ""
    marker.pose = pose2rospose(geometry.pose)
    marker.color.a = geometry.color.a
    marker.color.r = geometry.color.r
    marker.color.g = geometry.color.g
    marker.color.b = geometry.color.b
    marker.header.frame_id = frame_id
    marker.scale.x = geometry.scale.x
    marker.scale.y = geometry.scale.y
    marker.scale.z = geometry.scale.z
    marker.text = geometry.id #this cause a warning in rviz, but it can be helpful to see the id of the object
    return marker

def objects2markerarray(objects: list):
    '''
    convert object_map_server.interfaces.Object to visualization_msgs.MarkerArray
    '''
    marker_array = MarkerArray()
    for obj in objects:
        i = 0 #counter for id
        for _, geometry in obj.geometries.items():
            marker = geometry2marker(geometry, obj.name, obj.name, i)
            marker.id = i
            i += 1
            marker_array.markers.append(marker)
    return marker_array

def pose2rospose(pose):
    '''
    convert object_map_server.interfaces.Pose to geometry_msgs.Pose
    '''
    ros_pose = RosPose()
    ros_pose.position.x = pose.position.x
    ros_pose.position.y = pose.position.y
    ros_pose.position.z = pose.position.z
    q = tft.quaternion_from_euler(pose.orientation.roll, pose.orientation.pitch, pose.orientation.yaw)
    ros_pose.orientation.x = q[0]
    ros_pose.orientation.y = q[1]
    ros_pose.orientation.z = q[2]
    ros_pose.orientation.w = q[3]
    return ros_pose

def rospose2pose(pose):
    '''
    convert geometry_msgs.Pose to interfaces.Pose
    '''
    converted = Pose()
    converted.position.x = pose.position.x
    converted.position.y = pose.position.y
    converted.position.z = pose.position.z
    roll, pitch, yaw = tft.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    converted.orientation.roll = roll
    converted.orientation.pitch = pitch
    converted.orientation.yaw = yaw
    return converted
