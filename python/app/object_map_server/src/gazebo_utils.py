from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

SDF_VERSION = '1.5'
STATIC = True #makes SDF static to avoid gravity

from .interfaces import Object

def prettify(elem):
    rough_string = tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def object_as_sdf(object: Object) -> str:
    """Converts an object to an SDF string"""
    root = Element('sdf')
    root.set('version', SDF_VERSION)

    model = SubElement(root, 'model')
    model.set('name', object.name)

    # Make the model static
    if STATIC:
        static = SubElement(model, 'static')
        static.text = '1'

    for geometry in object.geometries.values():
        #skip bounding box
        if geometry.name == 'bb':
            continue

        link = SubElement(model, 'link')
        link.set('name', '{}_link'.format(geometry.name))

        # Make the model static
        if STATIC:
            static = SubElement(link, 'static')
            static.text = '1'

        pose = SubElement(link, 'pose')  # Add pose tag
        pose.text = '{} {} {} {} {} {}'.format(geometry.pose.position.x, 
                                               geometry.pose.position.y, 
                                               geometry.pose.position.z, 
                                               geometry.pose.orientation.roll,
                                               geometry.pose.orientation.pitch,
                                               geometry.pose.orientation.yaw)
        # Disable self collision
        self_collide = SubElement(link, 'self_collide')
        self_collide.text = '0'

        collision = SubElement(link, 'collision')
        collision.set('name', '{}_collision'.format(geometry.name))

        # Add geometry to collision
        geometry_collision = SubElement(collision, 'geometry')

        visual = SubElement(link, 'visual')
        visual.set('name', 'visual')

        geometry_visual = SubElement(visual, 'geometry')
        if geometry.type == 'CUBE':
            cube_collision = SubElement(geometry_collision, 'box')
            cube_visual = SubElement(geometry_visual, 'box')
            size = SubElement(cube_collision, 'size')
            size.text = '{} {} {}'.format(geometry.scale.x, geometry.scale.y, geometry.scale.z)
            size_visual = SubElement(cube_visual, 'size')
            size_visual.text = '{} {} {}'.format(geometry.scale.x, geometry.scale.y, geometry.scale.z)

        elif geometry.type == 'SPHERE':
            sphere_collision = SubElement(geometry_collision, 'sphere')
            sphere_visual = SubElement(geometry_visual, 'sphere')
            radius_collision = SubElement(sphere_collision, 'radius')
            radius_visual = SubElement(sphere_visual, 'radius')
            radius_collision.text = str(geometry.scale.x / 2)
            radius_visual.text = str(geometry.scale.x / 2)

        elif geometry.type == 'CYLINDER':
            cylinder_collision = SubElement(geometry_collision, 'cylinder')
            cylinder_visual = SubElement(geometry_visual, 'cylinder')
            radius_collision = SubElement(cylinder_collision, 'radius')
            radius_visual = SubElement(cylinder_visual, 'radius')
            radius_collision.text = str(geometry.scale.x / 2)
            radius_visual.text = str(geometry.scale.x / 2)
            length_collision = SubElement(cylinder_collision, 'length')
            length_visual = SubElement(cylinder_visual, 'length')
            length_collision.text = str(geometry.scale.z)
            length_visual.text = str(geometry.scale.z)

        elif geometry.type == 'MESH_RESOURCE':
            mesh_collision = SubElement(geometry_collision, 'mesh')
            uri_collision = SubElement(mesh_collision, 'uri')
            uri_collision.text = geometry.mesh_resource
            scale_collision = SubElement(mesh_collision, 'scale')
            scale_collision.text = '{} {} {}'.format(geometry.scale.x, geometry.scale.y, geometry.scale.z)

            mesh_visual = SubElement(geometry_visual, 'mesh')
            uri_visual = SubElement(mesh_visual, 'uri')
            uri_visual.text = geometry.mesh_resource
            scale_visual = SubElement(mesh_visual, 'scale')
            scale_visual.text = '{} {} {}'.format(geometry.scale.x, geometry.scale.y, geometry.scale.z)


        else:
            print('Marker type {} is not supported.'.format(geometry.type))

        sdf_string = prettify(root)
    return sdf_string




