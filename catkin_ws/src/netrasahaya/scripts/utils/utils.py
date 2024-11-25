import tf2_ros
import tf_conversions
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np

### Quaternion Related Functions ###

def quat_mult_point(q, w):
    """
    Multiply a quaternion by a point. Equivalent to the C++ method tf::quatMult
    :param q: the quaternion (x, y, z, w)
    :param w: the point (x, y, z)
    :return: the result of the multiplication
    """
    return (q[3] * w[0] + q[1] * w[2] - q[2] * w[1],
            q[3] * w[1] + q[2] * w[0] - q[0] * w[2],
            q[3] * w[2] + q[0] * w[1] - q[1] * w[0],
            -q[0] * w[0] - q[1] * w[1] - q[2] * w[2])

def quat_rotate(rotation, vector):
    """
    Rotate a vector according to a quaternion. Equivalent to the C++ method tf::quatRotate
    :param rotation: the rotation (x, y, z, w)
    :param vector: the vector to rotate (x, y, z)
    :return: the rotated vector (x, y, z)
    """

    q = quat_mult_point(rotation, vector)
    q = tf_conversions.transformations.quaternion_multiply(q, tf_conversions.transformations.quaternion_inverse(rotation))
    return [q[0], q[1], q[2]]

def get_relative_quaternion(q1, q2):
    """
    Get the relative quaternion between two quaternions, from q2 to q1
    :param q1: the first quaternion (x, y, z, w)
    :param q2: the second quaternion (x, y, z, w)
    :return: the relative quaternion from q2 to q1 (x, y, z, w)
    """

    # Note development: q1 untuk /rtabmap/localization_pose, q2 untuk /odom
    # cara mendapatkan relative quaternion ada dua:
    # 1. q1 / q2
    # 2. q1 * -q2 (q2 inverse/conjugation)

    q2_inv = tf_conversions.transformations.quaternion_inverse(q2)
    q_rot = tf_conversions.transformations.quaternion_multiply(q1, q2_inv)

    return q_rot

def get_euler_pose(pose: Pose):
    """
    Get the euler angles from a Pose message
    :param pose: the pose
    :return: the translation and rotation in the form (x, y, z, roll, pitch, yaw)
    """
    if not isinstance(pose, Pose):
        raise ValueError("pose must be a Pose")

    position = pose.position
    orientation = pose.orientation
    euler = tf_conversions.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    roll, pitch, yaw = euler
    return position.x, position.y, position.z, roll, pitch, yaw

### Geometry Related Functions ###

def align_pose(pose1: Pose, pose2: Pose) -> Pose:
    """
    Given two poses in the same frame, align the second pose to the first pose and return the relative pose (pose2 - pose1)
    :param pose1: the first pose
    :param pose2: the second pose
    :return: the relative pose (use it by multiplying your pose orientation with the relative orientation, and translate your pose position with the relative position)
    """
    # Note development: pose1 untuk /rtabmap/localization_pose, pose2 untuk /odom

    position1 = [pose1.position.x, pose1.position.y, pose1.position.z]
    orientation1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]

    position2 = [pose2.position.x, pose2.position.y, pose2.position.z]
    orientation2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]

    q_rot = get_relative_quaternion(orientation1, orientation2)

    relative_orientation = [q_rot[0], q_rot[1], q_rot[2], q_rot[3]]
    position_error = [position2[0] - position1[0], position2[1] - position1[1], position2[2] - position1[2]]

    return Pose(position=Point(x=position_error[0], y=position_error[1], z=position_error[2]), orientation=Quaternion(x=relative_orientation[0], y=relative_orientation[1], z=relative_orientation[2], w=relative_orientation[3]))

def rotate_point(point: Point, rx: float, ry: float, rz: float) -> Point:
    """
    Rotate a point around the origin according to the given angles
    :param point: the point to rotate
    :param rx: the rotation around the x axis
    :param ry: the rotation around the y axis
    :param rz: the rotation around the z axis
    """
    if not isinstance(point, Point):
        raise ValueError("point must be a Point")

    rot_matrix = tf_conversions.transformations.euler_matrix(rx, ry, rz, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_position = quat_rotate(q_rot, [point.x, point.y, point.z])

    return Point(x=new_position[0], y=new_position[1], z=new_position[2])

def local_to_global(pose: Pose, point: Point) -> Pose:
    """
    Convert a point in the local frame to the same frame as the pose relative to the pose as the origin
    :param pose: the pose
    :param point: the point
    :return: the point in the "global frame"
    """
    if not isinstance(pose, Pose):
        raise ValueError("pose must be a Pose")
    
    if not isinstance(point, Point):
        raise ValueError("point must be a Point")

    yaw = get_euler_pose(pose)[5]

    new_point = rotate_point(point, 0, 0, yaw)

    rotated_pose = Pose()
    rotated_pose.position.x = pose.position.x + new_point.x
    rotated_pose.position.y = pose.position.y + new_point.y
    rotated_pose.position.z = pose.position.z

    return rotated_pose

def world_to_grid(x, y, map_info: MapMetaData):
    """
    Convert a point in the world frame to the cell in the grid map
    :param x: the x coordinate
    :param y: the y coordinate
    :param map_info: the map metadata
    :return: the cell in the grid map
    """

    if not isinstance(map_info, MapMetaData):
        raise ValueError("map_info must be a MapMetaData")

    origin = map_info.origin.position
    resolution = map_info.resolution

    if resolution == 0:
        return 0, 0

    grid_x = int((x - origin.x) / resolution)
    grid_y = int((y - origin.y) / resolution)
    return grid_x, grid_y

def grid_to_world(grid_x, grid_y, map_info: MapMetaData):
    """
    Convert a cell in the grid map to a point in the world frame
    :param grid_x: the x coordinate in the grid
    :param grid_y: the y coordinate in the grid
    :param map_info: the map metadata
    :return: the point in the world frame
    """

    if not isinstance(map_info, MapMetaData):
        raise ValueError("map_info must be a MapMetaData")

    origin = map_info.origin.position
    resolution = map_info.resolution

    world_x = grid_x * resolution + origin.x
    world_y = grid_y * resolution + origin.y
    return world_x, world_y

### Algorithm Related Functions ###

def bresenham(x0, y0, x1, y1):
    """Bresenham's line algorithm."""
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points

def fill_polygon(polygon_edges):
    """Fills the interior of the polygon."""
    filled_points = set()
    # Group points by rows (y-coordinates)
    edge_points_by_row = {}
    for x, y in polygon_edges:
        edge_points_by_row.setdefault(y, []).append(x)
    
    # Sort each row's points by x-coordinate
    for y in edge_points_by_row:
        edge_points_by_row[y].sort()

    # Fill between edge pairs on each row
    for y, x_points in edge_points_by_row.items():
        for i in range(0, len(x_points) - 1, 2):
            x_start, x_end = x_points[i], x_points[i + 1]
            for x in range(x_start, x_end + 1):
                filled_points.add((x, y))
    
    return filled_points

def bresenham_polygon_with_fill(points):
    """
    Draws a polygon using Bresenham's line algorithm and fills the interior.
    Expects `points` to be a list of (x, y) tuples representing the vertices of the polygon.
    """
    # Step 1: Draw the polygon edges
    polygon_edges = []
    num_points = len(points)
    for i in range(num_points):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % num_points]  # Wraps around to the first point
        polygon_edges.extend(bresenham(x0, y0, x1, y1))

    # Step 2: Fill the polygon interior
    filled_points = fill_polygon(polygon_edges)

    # Combine edges and filled points
    return list(filled_points.union(polygon_edges))

### Extra Functions ###

def occupancygrid_to_numpy(msg: OccupancyGrid):
    """
    Convert an OccupancyGrid message to a numpy array
    """
    if not isinstance(msg, OccupancyGrid):
        raise ValueError("msg must be an OccupancyGrid")
    
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    return np.ma.array(data, mask=data==-1, fill_value=-1)