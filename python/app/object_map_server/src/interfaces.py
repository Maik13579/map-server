import os
import yaml
import shutil
from typing import Optional, Set, Tuple, List, Dict

POSE_FILE_NAME = 'pose.yaml'
GEOMETRY_FILE_NAME = 'geometry.yaml'

class Vector3:
    """Class to represent a vector in 3D space."""

    def __init__(self, x: float=0.0, y: float=0.0, z: float=0.0):
        """Initialize a vector instance.
        Args:
            x, y, z: Coordinate values.
        """
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        '''
        defines string conversion
        '''
        return "("+str(self.x)+", "+str(self.y)+", "+str(self.z)+")"


class Orientation:
    """Class to represent an orientation in 3D space."""

    def __init__(self, roll: float=0.0, pitch: float=0.0, yaw: float=0.0):
        """Initialize an Orientation instance.
        Args:
            roll, pitch, yaw: Euler angles.
        """
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __str__(self):
        '''
        defines string conversion
        '''
        return "("+str(self.roll)+", "+str(self.pitch)+", "+str(self.yaw)+")"


class Pose:
    """Class to represent a pose in 3D space."""

    def __init__(self, position: Vector3=None, orientation: Orientation=None):
        self.position = position if position is not None else Vector3()
        self.orientation = orientation if orientation is not None else Orientation()

    def __str__(self):
        '''
        defines string conversion
        '''
        return "\n  position: "+str(self.position)+"\n  orientation: "+str(self.orientation)


    def get_dict(self):
        """Get a dictionary representation of this Pose.

        Returns:
            dict: Dictionary representation of this Pose.
        """
        return {
            'position': {
                'x': self.position.x,
                'y': self.position.y,
                'z': self.position.z
            },
            'orientation': {
                'roll': self.orientation.roll,
                'pitch': self.orientation.pitch,
                'yaw': self.orientation.yaw
            }
        }
    
class Color:
    '''
    defines a RGBA color
    '''
    def __init__(self, r: float=0.0, g: float=0.0, b: float=0.0, a: float=1.0) -> None:
        self.r = 1.0
        self.g = 1.0
        self.b = 1.0
        self.a = 1.0

    def __str__(self):
        '''
        defines string conversion
        '''
        return "("+str(self.r)+", "+str(self.g)+", "+str(self.b)+", "+str(self.a)+")"

class Geometry:
    '''
    defines a geometry
    different types:
        CUBE: pose is the center
        SPHERE: pose is the center
            scale.x is diameter in x direction, scale.y in y direction, scale.z in z direction.
            By setting these to different values you get an ellipsoid instead of a sphere. 
        CYLINDER: pose is the center
            scale.x is diameter in x direction, scale.y in y direction. 
            By setting these to different values you get an ellipse instead of a circle.
            Use scale.z to specify the height. 
        MESH_RESOURCE: use mesh_resource as the path to the mesh file
            Use scale to change the size of the mesh
    '''
    def __init__(self, name: str, type: str, mesh_resource: str="") -> None:
        if type not in ["CUBE", "SPHERE", "CYLINDER", "MESH_RESOURCE"]:
            raise Exception("type not known!")
        self.name = name
        self.pose = Pose()
        self.scale = Vector3()
        self.color = Color()
        self.type = type
        self.mesh_resource = mesh_resource

    def __str__(self):
        '''
        defines string conversion
        '''
        return "Name: "+str(self.name)+"\ntype: "+str(self.type)+"\npose: "+str(self.pose)+"\nscale: "+str(self.scale)+"\ncolor: "+str(self.color)

class Object:
    '''
    defines a object
    geometries are the basic shapes the object consist off
    name is a unique identifier
    ns is the namespace
    '''
    def __init__(self, name: str, ns: str="", geometries: Dict[str, Geometry]=None) -> None:
        self.name = name
        self.ns = ns
        self.geometries = geometries if geometries is not None else {}

    def __str__(self):
        '''
        defines string conversion
        '''
        return os.path.join(self.ns, str(self.name))

class Frame:
    """Class to represent a directory in the file system tree."""

    def __init__(self, name: str, parent: Optional['Frame'], pose: Pose=None):
        """
        Initialize a Frame instance.
        Args:
            name: Name of the directory this Frame represents.
            parent: Parent Frame of this Frame. Path to this directory for root Frame
            pose: The position of the Frame realativ to parent Frame and the orientation of the Frame in Euler angles.
        """
        self.name = name
        self.parent = parent
        self.children = []  # List to store child Frames (subdirectories).
        self.pose = pose if pose is not None else Pose()

    def __str__(self, level: int = 0):
        """Print the file system tree structure along with the pose information.

        Args:
            level: Depth level of the current Frame (used for indentation).

        Returns:
            str: String representation of the Frame tree.
        """
        indent = ' ' * level
        pose = self.pose
        position = pose.position
        orientation = pose.orientation
        s = (f"{indent}Frame: {self.name}\n"
             f"{indent}  Position: x={position.x}, y={position.y}, z={position.z}\n"
             f"{indent}  Orientation: roll={orientation.roll}, pitch={orientation.pitch}, yaw={orientation.yaw}\n")
        for child in self.children:
            s += child.__str__(level + 2)
        return s
    
    def get_path(self) -> str:
        """Get the path of this Frame.

        Returns:
            str: Path of this Frame.
        """
        if isinstance(self.parent, str):
            return os.path.join(self.parent, self.name)
        else:
            return os.path.join(self.parent.get_path(), self.name)

    def find_frame(self, frame_name: str) -> Optional[Tuple['Frame', str]]:
        """Recursively search for a frame by name in this frame's descendants.

        Args:
            frame_name: Name of the frame to search for.

        Returns:
            Tuple containing the Frame and the path to the frame as a string, or None if the frame was not found.
        """
        if frame_name == self.name:
            return self, self.name
        
        for child in self.children:
            if child.name == frame_name:
                return child, os.path.join(self.name, child.name)
            else:
                found_frame, found_frame_path = child.find_frame(frame_name)
                if found_frame is not None:
                    return found_frame, os.path.join(self.name, found_frame_path)
        return None, None

    def move_frame(self, frame_name: str, new_parent_name: str):
        """Move a frame to a new parent frame.

        Args:
            root_frame: Root Frame of the file system tree.
            frame_name: Name of the frame to move.
            new_parent_name: Name of the new parent frame.
            path_prefix: Prefix to add to the path of the frame to move.
        """
        frame, frame_path = self.find_frame(frame_name)
        if not frame:
            print(f"Frame {frame_name} not found.")
            return None

        frame_path = os.path.join(self.parent, frame_path)

        new_parent_frame, parent_frame_path = self.find_frame(new_parent_name)
        if not new_parent_frame:
            print(f"Parent frame {new_parent_name} not found.")
            return None

        parent_frame_path = os.path.join(self.parent, parent_frame_path)
        new_frame_path = os.path.join(parent_frame_path, frame.name)

        print('moving frame from', frame_path, 'to', new_frame_path)
        try:
            # Use shutil.move to move the directory.
            shutil.move(frame_path, new_frame_path)

            # If successful, update the frame's parent and update children list of the old and new parents.
            frame.parent.children.remove(frame)
            frame.parent = new_parent_frame
            new_parent_frame.children.append(frame)
            
            return frame
        except Exception as e:
            print(f"Failed to move frame {frame.name} to {new_frame_path}. Error: {e}")
            return None
        
    def move(self, new_parent_name: str):
        """Move a frame to a new parent frame.

        Args:
            root_frame: Root Frame of the file system tree.
            new_parent_name: Name of the new parent frame.
            path_prefix: Prefix to add to the path of the frame to move.
        """
        new_parent_frame, parent_frame_path = self.find_frame(new_parent_name)
        if not new_parent_frame:
            print(f"Parent frame {new_parent_name} not found.")
            return None

        parent_frame_path = os.path.join(self.parent, parent_frame_path)
        new_frame_path = os.path.join(parent_frame_path, self.name)

        print('moving frame from', self.get_path(), 'to', new_frame_path)
        try:
            # Use shutil.move to move the directory.
            shutil.move(self.get_path(), new_frame_path)

            # If successful, update the frame's parent and update children list of the old and new parents.
            self.parent.children.remove(self)
            self.parent = new_parent_frame
            new_parent_frame.children.append(self)
            
            return self
        except Exception as e:
            print(f"Failed to move frame {self.name} to {new_frame_path}. Error: {e}")
            return None
    
    def add_frame_to(self, frame_name: str, parent_frame_name: str, pose: Pose = Pose(Vector3(), Orientation())):
        """Create a new child frame and corresponding directory.

        Args:
            child_name: Name of the new child frame.
            parent_frame_name: Name of the parent frame.
            pose: Pose of the new child frame.

        Returns:
            Frame: The new child frame.
        """
        print(f"Adding frame {frame_name} to {parent_frame_name}")
        parent_frame, _ = self.find_frame(parent_frame_name)
        if not parent_frame:
            print(f"Parent frame {parent_frame_name} not found.")
            return None

        new_frame = Frame(frame_name, parent_frame, pose)
        parent_frame.children.append(new_frame)

        #Check if the new frame's name is unique.
        if self.has_duplicate_names():
            print(f"A frame named {frame_name} already exists.")
            parent_frame.children.remove(new_frame)
            return None

        # Create a new directory for the child frame.
        child_path = os.path.join(parent_frame.get_path(), frame_name)
        try:
            # Create a new directory for the child frame.
            os.makedirs(child_path, exist_ok=True)

            # Create pose.yaml file in the new directory.
            with open(os.path.join(child_path, POSE_FILE_NAME), 'w') as f:
                yaml.dump(pose.get_dict(), f)

            return new_frame
        except Exception as e:
            print(f"Failed to add child frame {frame_name} to {parent_frame_name}. Error: {e}")
            return None
        
    def add_frame(self, frame_name: str, pose: Pose = Pose(Vector3(), Orientation())):
        """Create a new child frame and corresponding directory.

        Args:
            child_name: Name of the new child frame.
            pose: Pose of the new child frame.

        Returns:
            Frame: The new child frame.
        """
        print(f"Adding frame {frame_name} to {self.name}")

        new_frame = Frame(frame_name, self, pose)
        self.children.append(new_frame)

        #Check if the new frame's name is unique.
        if self.has_duplicate_names():
            print(f"A frame named {frame_name} already exists.")
            self.children.remove(new_frame)
            return None

        # Create a new directory for the child frame.
        child_path = os.path.join(self.get_path(), frame_name)
        try:
            # Create a new directory for the child frame.
            os.makedirs(child_path, exist_ok=True)

            # Create pose.yaml file in the new directory.
            with open(os.path.join(child_path, POSE_FILE_NAME), 'w') as f:
                yaml.dump(pose.get_dict(), f)

            return new_frame
        except Exception as e:
            print(f"Failed to add child frame {frame_name} to {self.name}. Error: {e}")
            return None
        
    def change_pose_of(self, frame_name: str, pose: Pose):
        """Change the pose of a frame.

        Args:
            frame_name: Name of the frame to change the pose of.
            pose: New pose of the frame.
        """
        frame, _ = self.find_frame(frame_name)
        if not frame:
            print(f"Frame {frame_name} not found.")
            return None

        try:
            # Update the pose.yaml file.
            with open(os.path.join(frame.get_path(), POSE_FILE_NAME), 'w') as f:
                yaml.dump(pose.get_dict(), f)

            # Update the frame's pose.
            frame.pose = pose
        except Exception as e:
            print(f"Failed to change pose of frame {frame_name}. Error: {e}")
            return None
        
    def change_pose(self, pose: Pose):
        """Change the pose of a frame.

        Args:
            pose: New pose of the frame.
        """

        try:
            # Update the pose.yaml file.
            with open(os.path.join(self.get_path(), POSE_FILE_NAME), 'w') as f:
                yaml.dump(pose.get_dict(), f)

            # Update the frame's pose.
            self.pose = pose
        except Exception as e:
            print(f"Failed to change pose of frame {self.name}. Error: {e}")
            return None
        
    def has_duplicate_names(self) -> bool:
        """Check if there are duplicate frame names in the file system tree.

        Args:
            root_frame: Root Frame of the file system tree.

        Returns:
            True if there are duplicate frame names, False otherwise.
        """
        def _dfs(frame: Frame, seen_names: Set[str]) -> bool:
            """Perform a depth-first search to find duplicate frame names.

            Args:
                frame: Current Frame in the DFS.
                seen_names: Set of frame names seen so far in the DFS.

            Returns:
                True if a duplicate frame name is found, False otherwise.
            """
            if frame.name in seen_names:
                return True
            seen_names.add(frame.name)
            for child in frame.children:
                if _dfs(child, seen_names):
                    return True
            return False

        return _dfs(self, set())


def load_objects(path: str) -> List[Object]:
    """Load all objects in the given path.
    
     Args:
        path: File system path to the objects.

    Returns:
        List of Objects.
    """
    #check if path exists
    if not os.path.exists(path):
        raise ValueError("Path does not exist.")
    
    objects = []
    for root, _, files in os.walk(path):
        if GEOMETRY_FILE_NAME in files:
            parent_dir = os.path.dirname(root)
            object_ns = os.path.relpath(parent_dir, path) if parent_dir != path else ""
            object_name = os.path.basename(root)
            print(f"Loading object {os.path.join(object_ns, object_name)}")

            with open(os.path.join(root, GEOMETRY_FILE_NAME), 'r') as f:
                geometries = yaml.load(f, Loader=yaml.Loader)
            
            objects.append(Object(object_name, object_ns, geometries))
    return objects



def load_frames(path: str) -> Frame:
    """Load a file system tree from the given path.

    Args:
        path: File system path to load the file system tree from.

    Returns:
        Root Frame of the loaded file system tree.
    """
    #check if path exists
    if not os.path.exists(path):
        raise ValueError("Path does not exist.")
    
    parent_path, frame_name = path.rsplit('/',1)
    root_frame = Frame(frame_name, parent_path)  # Initialize root Frame.

    print('Loading file system tree...')
    _load_frames_recursiv(root_frame, path)  # Load file system tree.

    # Check whether there are duplicate frame names.
    if root_frame.has_duplicate_names():
        raise ValueError("There are two or more frames with the same name.")

    print(root_frame)  # Print file system tree.
    return root_frame

def _load_frames_recursiv(parent_frame: Frame, path: str):
    """Recursively load directories from the file system into Frame objects.

    Args:
        parent_frame: Frame object to which loaded directories are added as children.
        path: File system path to load directories from.
    """
    for dirpath, dirnames, filenames in os.walk(path):
        # Look for the POSE_FILE_NAME file in the current directory.
        if POSE_FILE_NAME in filenames:
            pose_path = os.path.join(dirpath, POSE_FILE_NAME)
            with open(pose_path, 'r') as f:
                data = yaml.safe_load(f)
                position_data = data.get('position', {})
                orientation_data = data.get('orientation', {})
                position = Vector3(position_data.get('x', 0.0), 
                                    position_data.get('y', 0.0), 
                                    position_data.get('z', 0.0))
                orientation = Orientation(orientation_data.get('roll', 0.0), 
                                          orientation_data.get('pitch', 0.0), 
                                          orientation_data.get('yaw', 0.0))
                parent_frame.pose = Pose(position, orientation)
        else:
            # If POSE_FILE_NAME is not found, use default pose.
            default_pose = Pose(Vector3(), Orientation())
            parent_frame.pose = default_pose

            # Create pose.yaml file in the new directory.
            with open(os.path.join(dirpath, POSE_FILE_NAME), 'w') as f:
                yaml.dump(default_pose.get_dict(), f)

        for dirname in dirnames:
            full_dir_path = os.path.join(dirpath, dirname)
            child_frame = Frame(dirname, parent_frame, Pose(Vector3(), Orientation()))
            parent_frame.children.append(child_frame)
            _load_frames_recursiv(child_frame, full_dir_path)
        break  # This is required to limit os.walk to one level deep.