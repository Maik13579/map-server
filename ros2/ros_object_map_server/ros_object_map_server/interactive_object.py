from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, Pose

from ros_object_map_server.conversion import object2markerarray, pose2rospose, rospose2pose, compose_poses

class InteractiveObject:
    def __init__(self, server, obj, frame):
        self.obj = obj
        self.frame = frame
        self.moveable = False
        self.server = server

        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = frame.name
        self.int_marker.name = obj.name
        self.int_marker.pose.orientation.w = 1.0

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True

        #get geometries of objects
        control.markers += object2markerarray(self.obj).markers
        #add axis
        control.markers += create_axis_marker(obj, 0.5)
        self.int_marker.controls.append(control)

    def update_callback(self, feedback):
        self.update_pose = feedback.pose
 
        self.server.setPose(feedback.marker_name, self.update_pose)
        self.server.applyChanges()


    def click(self, feedback):
        print("\nclicked on:")
        print(self.obj)

    def move(self, feedback):
        #Toggle moveable
        if self.moveable:
            self.disable_move()
        else:
            self.enable_move()

    def disable_move(self):
        #reset interactive marker pose
        self.int_marker.pose = Pose()
        self.int_marker.pose.orientation.w = 1.0

        #update object pose
        update_pose = compose_poses(pose2rospose(self.frame.pose), self.update_pose)
        self.frame.pose = rospose2pose(update_pose)



        self.moveable = False
        #remove all 6 dof controls
        controls = []
        for control in self.int_marker.controls:
            if control.name[:6] != "6_dof_":
                controls.append(control)
        self.int_marker.controls = controls

    def enable_move(self):
        #init pose
        self.update_pose = Pose()
        self.update_pose.orientation.w = 1.0

        self.moveable = True
        #add 6 dof controls
        self.int_marker.controls += get_6_dof_controls()

    def add_highlight(self, a=0.1):
        for marker in self.int_marker.controls[0].markers:
            marker.color.a = a if marker.text != 'bb' else 0.0 #ignore bounding box

    def remove_highlight(self):
        for marker in self.int_marker.controls[0].markers:
            marker.color.a = 1.0 if marker.text != 'bb' else 0.0 #ignore bounding box

def create_axis_marker(obj, size=0.1):
    markers = []
    # X-axis cylinder
    x_marker = Marker()
    x_marker.type = Marker.CUBE
    x_marker.scale.x = size
    x_marker.scale.y = size/10
    x_marker.scale.z = size/10
    x_marker.color.r = 1.0
    x_marker.color.g = 0.0
    x_marker.color.b = 0.0
    x_marker.color.a = 1.0
    x_marker.pose.orientation.w = 1.0
    x_marker.pose.position.x = size/2
    markers.append(x_marker)
    # Y-axis cylinder
    y_marker = Marker()
    y_marker.type = Marker.CUBE
    y_marker.scale.x = size/10
    y_marker.scale.y = size
    y_marker.scale.z = size/10
    y_marker.color.r = 0.0
    y_marker.color.g = 1.0
    y_marker.color.b = 0.0
    y_marker.color.a = 1.0
    y_marker.pose.orientation.w = 1.0
    y_marker.pose.position.y = size/2
    markers.append(y_marker)
    # Z-axis cylinder
    z_marker = Marker()
    z_marker.type = Marker.CUBE
    z_marker.scale.x = size/10
    z_marker.scale.y = size/10
    z_marker.scale.z = size
    z_marker.color.r = 0.0
    z_marker.color.g = 0.0
    z_marker.color.b = 1.0
    z_marker.color.a = 1.0
    z_marker.pose.orientation.w = 1.0
    z_marker.pose.position.z = size/2
    markers.append(z_marker)
    text_marker = Marker()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.scale.z = 0.2
    text_marker.pose.position.z = size
    text_marker.color.r = 1.0
    text_marker.color.g = 1.0
    text_marker.color.b = 1.0
    text_marker.color.a = 1.0
    text_marker.text = obj.name
    markers.append(text_marker)
    # arrow_marker = Marker()
    # arrow_marker.type = Marker.ARROW
    # arrow_marker.scale.x = 0.01
    # arrow_marker.scale.y = 0.01
    # arrow_marker.scale.z = 0.01
    # arrow_marker.color.r = 1.0
    # arrow_marker.color.g = 1.0
    # arrow_marker.color.b = 0.0
    # arrow_marker.color.a = 1.0
    # arrow_marker.points.append(Point(0, 0, 0))
    # arrow_marker.points.append(Point(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z))
    # arrow_marker.pose = pose2rospose(obj.pose)
    # arrow_marker.pose.orientation.x = -arrow_marker.pose.orientation.x
    # arrow_marker.pose.orientation.y = -arrow_marker.pose.orientation.y
    # arrow_marker.pose.orientation.z = -arrow_marker.pose.orientation.z
    # arrow_marker.pose.orientation.w = -arrow_marker.pose.orientation.w
    # arrow_marker.pose.position.x = -arrow_marker.pose.position.x
    # arrow_marker.pose.position.y = -arrow_marker.pose.position.y
    # arrow_marker.pose.position.z = -arrow_marker.pose.position.z
    # markers.append(arrow_marker)
    return markers

def get_6_dof_controls():
    controls = []
    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.name = "6_dof_rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 1.0
    control.orientation.y = 0.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.name = "6_dof_move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.name = "6_dof_rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 1.0
    control.orientation.z = 0.0
    normalizeQuaternion(control.orientation)
    control.name = "6_dof_move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    normalizeQuaternion(control.orientation)
    control.name = "6_dof_rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1.0
    control.orientation.x = 0.0
    control.orientation.y = 0.0
    control.orientation.z = 1.0
    normalizeQuaternion(control.orientation)
    control.name = "6_dof_move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    controls.append(control)
    return controls

def normalizeQuaternion(quaternion_msg):
    norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
    s = norm**(-0.5)
    quaternion_msg.x *= s
    quaternion_msg.y *= s
    quaternion_msg.z *= s
    quaternion_msg.w *= s
