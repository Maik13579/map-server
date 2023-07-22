import numpy as np
from scipy.spatial.transform import Rotation

import PIL.ImageDraw as ImageDraw
import PIL.Image as Image

from typing import List, Tuple

def cube_plane_intersection(center: np.ndarray, pose: np.ndarray, scale: List[float], 
                            plane_normal: np.ndarray, plane_point: np.ndarray) -> List[np.ndarray]:
    """
    Compute the intersection points of a cube and a plane.

    Args:
        center (np.ndarray): The center of the cube.
        pose (np.ndarray): The pose (orientation) of the cube represented by a quaternion.
        scale (List[float]): The scale (size) of the cube.
        plane_normal (np.ndarray): The normal vector of the plane.
        plane_point (np.ndarray): A point on the plane.

    Returns:
        list: A list of intersection points (np.ndarray) of the cube and the plane.
    """
    # convert quaternion to rotation matrix
    rotation = Rotation.from_quat(pose).as_matrix()

    half_lengths = np.array(scale) / 2
    
    vertices = np.array([
        [-1, -1, -1],
        [-1, -1, 1],
        [-1, 1, -1],
        [-1, 1, 1],
        [1, -1, -1],
        [1, -1, 1],
        [1, 1, -1],
        [1, 1, 1],
    ]) * half_lengths
    vertices = vertices @ rotation.T + center

    edges = [
        (0, 1), (0, 2), (0, 4),
        (1, 3), (1, 5),
        (2, 3), (2, 6),
        (3, 7),
        (4, 5), (4, 6),
        (5, 7),
        (6, 7),
    ]
    intersections = []
    for i, j in edges:
        v1, v2 = vertices[i], vertices[j]
        t = np.dot(plane_point - v1, plane_normal) / np.dot(v2 - v1, plane_normal)
        if 0 <= t <= 1:
            intersections.append(v1 + t * (v2 - v1))
    
    return intersections

def cylinder_plane_intersection(center: np.ndarray, pose: np.ndarray, height: float, radius: float, 
                                plane_normal: np.ndarray, plane_point: np.ndarray) -> List[np.ndarray]:
    """
    Compute the intersection points of a cylinder and a plane.

    Args:
        center (np.ndarray): The center of the cylinder.
        pose (np.ndarray): The pose (orientation) of the cylinder represented by a quaternion.
        height (float): The height of the cylinder.
        radius (float): The radius of the cylinder.
        plane_normal (np.ndarray): The normal vector of the plane.
        plane_point (np.ndarray): A point on the plane.

    Returns:
        list: A list of intersection points (np.ndarray) of the cylinder and the plane.
    """
    # convert quaternion to rotation matrix
    rotation = Rotation.from_quat(pose).as_matrix()

    # Compute the two center points of the top and bottom faces of the cylinder
    axis_point1 = np.array([0, 0, -height/2])
    axis_point2 = np.array([0, 0, height/2])

    # Rotate the points according to the pose and move them back to the original location
    axis_point1 = (axis_point1 @ rotation.T) + center
    axis_point2 = (axis_point2 @ rotation.T) + center

    # Compute the intersection of the plane with the cylinder's axis
    intersections = []
    t = np.dot(plane_point - axis_point1, plane_normal) / np.dot(axis_point2 - axis_point1, plane_normal)
    if 0 <= t <= 1:
        intersections.append(axis_point1 + t * (axis_point2 - axis_point1))

    # Check the intersection of the plane with the top and bottom circles of the cylinder
    for z in [-height/2, height/2]:
        circle_center = np.array([0, 0, z])
        for angle in np.linspace(0, 2 * np.pi, 100):  # discretize the circle
            point_on_circle = circle_center + radius * np.array([np.cos(angle), np.sin(angle), 0])
            
            # Apply rotation and move the point back to the original location
            point_on_circle = (point_on_circle @ rotation.T) + center

            point_above_circle = point_on_circle + np.array([0, 0, 1])  # point slightly above the circle
            t = np.dot(plane_point - point_on_circle, plane_normal) / np.dot(point_above_circle - point_on_circle, plane_normal)
            if 0 <= t <= 1:
                intersections.append(point_on_circle + t * (point_above_circle - point_on_circle))

    return intersections


def intersection_to_grid_map(intersections: List[List[np.ndarray]], map_size: Tuple[float, float]=(10.0, 10.0), 
                             grid_size: float=0.04) -> Tuple[Image.Image, int, int]:
    """
    Convert the intersection points to a grid map image.

    Args:
        intersections (List[List[np.ndarray]]): A list of intersections where each intersection is a list of points.
        map_size (Tuple[float, float], optional): The size of the map (width, height). Defaults to (10.0, 10.0).
        grid_size (float, optional): The size of the grid. Defaults to 0.04.

    Returns:
        Tuple[Image.Image, int, int]: A tuple containing the grid map image, width of the image, and height of the image.
    """
    polygons = []
    single_points = []
    for intersection in intersections:
        points = []
        for point in intersection:
            points.append((point[0]/grid_size, point[1]/grid_size))
        
        if len(points) < 2:
            single_points.append(points[0])
        else:
            points[0], points[1] = points[1], points[0]
            polygons.append(points)

    width = int(20/grid_size)
    height = int(20/grid_size)

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image)
    for points in polygons:
        new_points = []
        for point in points:
            point = list(point)
            point[0] += width/2
            point[1] += height/2
            new_points.append((point[0], point[1]))
        draw.polygon((new_points), fill="black")
    
    for point in single_points:
        point = list(point)
        point[0] += width/2
        point[1] += height/2
        draw.point(point, fill="black")
    
    image = image.convert('L')
    #image.save("/objects/test.png")
    return image, width, height
