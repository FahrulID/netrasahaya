import tf2_ros
import tf_conversions
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import Point as ShapelyPoint
import rospy
from collections import deque
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid

def check_passability(start, end_points, map_data, threshold=50):
    if not end_points:
        return False
        
    visited = set()
    queue = deque([start])
    visited.add(start)
    
    while queue:
        x, y = queue.popleft()
        if (x, y) in end_points:
            return True
            
        for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
            nx, ny = x + dx, y + dy
            if (0 <= ny < map_data.shape[0] and 0 <= nx < map_data.shape[1] and
                (nx, ny) not in visited and map_data[ny][nx] > -2 and map_data[ny][nx] < threshold):
                queue.append((nx, ny))
                visited.add((nx, ny))
    
    return False

def check_nearest_occupied(start, map_data, map_info: MapMetaData, threshold=50):
    if not isinstance(map_info, MapMetaData):
        raise ValueError("map_info must be a MapMetaData")
    
    visited = set()
    # Include diagonal movements
    directions = [(0,1), (1,0), (0,-1), (-1,0), 
                 (1,1), (1,-1), (-1,1), (-1,-1)]
    queue = deque([(start, 0)])  # (position, distance)
    visited.add(start)
    min_dist = float('inf')
    
    while queue:
        (x, y), curr_dist = queue.popleft()
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if (0 <= ny < map_data.shape[0] and 0 <= nx < map_data.shape[1] and
                (nx, ny) not in visited):
                
                # Calculate actual Euclidean distance from start
                real_dist = math.sqrt((nx - start[0])**2 + (ny - start[1])**2)
                
                if map_data[ny][nx] >= threshold:
                    real_world_dist = real_dist * map_info.resolution
                    min_dist = min(min_dist, real_world_dist)
                    continue
                    
                if map_data[ny][nx] > -2:
                    queue.append(((nx, ny), curr_dist + 1))
                    visited.add((nx, ny))
    
    return min_dist if min_dist != float('inf') else -1

def get_grid_section(map_data: OccupancyGrid, cell_coords: list, start_point: list, end_points: list):
    """
    Extract and format a section of occupancy grid based on given cell coordinates
    
    Args:
        msg: OccupancyGrid message
        cell_coords: List of tuples containing (x,y) cell coordinates
    
    Returns:
        numpy array with specified section, other cells marked as 100
    """

    # get cell grids bounding box of cell_coords
    min_x = int(min(cell_coords, key=lambda x: x[0])[0])
    max_x = int(max(cell_coords, key=lambda x: x[0])[0])
    min_y = int(min(cell_coords, key=lambda x: x[1])[1])
    max_y = int(max(cell_coords, key=lambda x: x[1])[1])

    width = max_x - min_x + 1
    height = max_y - min_y + 1

    # Create new array filled with -2
    formatted_grid = np.full((height, width), -2, dtype=np.int8)

    # Convert cell coordinates to numpy array for easier handling
    coords = np.array(cell_coords)

    # Copy values from original grid for specified coordinates
    for x, y in coords:
        x = int(x)
        y = int(y)
        grid_index = grid_to_index(x, y, map_data.info)
        if grid_index < 0 or grid_index >= len(map_data.data):
            continue

        formatted_grid[y - min_y, x - min_x] = map_data.data[grid_index]

    # transform start and end points to new grid
    start_point = (start_point[0] - min_x, start_point[1] - min_y)
    end_points = [(x - min_x, y - min_y) for x, y in end_points]

    # loop over the formatted grid for visualization
    # for i in range(height):
    #     for j in range(width):
    #         print(f'{chr(94 + int(formatted_grid[i, j]))}', end='')
    #     print()

    return formatted_grid, start_point, end_points

def get_polygon_points(pose, theta, theta_range, max_distance, info):
    if not isinstance(pose, Pose):
        raise ValueError("pose must be a Pose")
    
    if not isinstance(info, MapMetaData):
        raise ValueError("info must be a MapMetaData")
    
    yaw = tf_conversions.transformations.euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])[2]

    start_angle = yaw + theta - theta_range/2
    end_angle = yaw + theta + theta_range/2

    start_point = world_to_grid(pose.position.x, pose.position.y, info)
    end_points = []

    vertices = [] # center, leftmost, rightmost
    vertices.append((pose.position.x, pose.position.y))

    edge_points = []

    for angle in np.linspace(start_angle, end_angle):
        x = pose.position.x + max_distance * math.cos(angle)
        y = pose.position.y + max_distance * math.sin(angle)
        grid_x, grid_y = world_to_grid(x, y, info)
        end_points.append((grid_x, grid_y))
        edge_points.append((grid_x, grid_y))

        # first point
        if angle == start_angle:
            vertices.append((grid_x, grid_y))
        # leftmost point
        if angle == end_angle:
            vertices.append((grid_x, grid_y))

    # right edge
    edge_points.extend(bresenham_line(vertices[2], start_point))
    # left edge
    edge_points.extend(bresenham_line(start_point, vertices[1]))

    inside_points = get_grid_points_inside_polygon(edge_points)

    return start_point, end_points, vertices, edge_points, inside_points

def bresenham_line(start_point, end_point):
    """Get all points in line using Bresenham's algorithm with numpy
    Args:
        start_point: tuple (x1, y1)
        end_point: tuple (x2, y2)
    Returns:
        Array of points [(x,y), ...]
    """
    x1, y1 = start_point
    x2, y2 = end_point
    
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    
    x = x1
    y = y1
    
    x_inc = 1 if x2 > x1 else -1
    y_inc = 1 if y2 > y1 else -1
    
    points = []
    points.append((x, y))
    
    if dx > dy:
        error = dx / 2
        while x != x2:
            error -= dy
            if error < 0:
                y += y_inc
                error += dx
            x += x_inc
            points.append((x, y))
    else:
        error = dy / 2
        while y != y2:
            error -= dx
            if error < 0:
                x += x_inc
                error += dy
            y += y_inc
            points.append((x, y))
            
    return points

def get_polygon_segment_from_pose(pose: Pose, distance=1.0, theta=0.0, theta_range=0.0):
    """
    Get the polygon segment from a pose
    :param pose: the pose
    :param distance: the distance of the segment
    :param theta: the angle of the segment
    :param theta_range: the range of the angle
    :return: the polygon segment consisting of list of far polygon and near polygon
      P2------P3
        \    |
     HP2 \___| HP3
          \  |
           \ |
            \| P1
    """
    point1 = Point(x=0, y=0, z=0)
    point2 = rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta + theta_range)
    point3 = rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta - theta_range)
    halfpoint2 = rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta + theta_range)
    halfpoint3 = rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta - theta_range)

    point1_pose = local_to_global(pose, point1)
    point2_pose = local_to_global(pose, point2)
    point3_pose = local_to_global(pose, point3)
    halfpoint2_pose = local_to_global(pose, halfpoint2)
    halfpoint3_pose = local_to_global(pose, halfpoint3)

    near_polygon = [(point1_pose.position.x, point1_pose.position.y), (halfpoint2_pose.position.x, halfpoint2_pose.position.y), (halfpoint3_pose.position.x, halfpoint3_pose.position.y)]
    far_polygon = [(halfpoint2_pose.position.x, halfpoint2_pose.position.y), (point2_pose.position.x, point2_pose.position.y), (point3_pose.position.x, point3_pose.position.y), (halfpoint3_pose.position.x, halfpoint3_pose.position.y)]

    return near_polygon, far_polygon

def is_path_possible(grid, start_points, end_points, threshold=10) -> bool:
    """
    Utilizing BFS to find if there is a path from any cell in the first row to any cell in the last row
    :param grid: the grid
    :param start_points: the starting points (list of (x, y) tuples)
    :param end_points: the ending points (list of (x, y) tuples)
    :param boundary_polygon: the boundary polygon
    :return: True if there is a path, False otherwise
    """
    rows, cols = grid.shape

    if not start_points:
        return False  # No free starting points in the last row
    
    # Directions for 4-connected grid (up, down, left, right)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    # BFS queue and visited set
    queue = deque(start_points)
    visited = set(start_points)

    while queue:
        y, x = queue.popleft()

        # If we reach one of the points then return True
        if (x, y) in end_points:
            return True

        # Explore neighbors
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            inside_boundary = 0 <= nx < rows and 0 <= ny < cols
            if not inside_boundary:
                continue

            is_visited = (nx, ny) in visited
            if is_visited:
                continue

            is_occupied = grid[nx, ny] >= threshold
            if is_occupied:
                continue

            queue.append((nx, ny))
            visited.add((nx, ny))

    return False  # No path found

def check_if_points_in_grid_map_are_occupied(points, map_data, threshold=0):
    """
    Check if the points in the grid map are occupied
    :param points: the points to check
    :param map_data: the map data
    :param threshold: the threshold to consider a cell as occupied
    :return: True if all points are occupied, False otherwise
    """
    for x, y in points:
        if x < 0 or y < 0 or x >= map_data.shape[1] or y >= map_data.shape[0]:
            continue
        if map_data[y, x] > threshold:
            return True
    return False

def get_grid_points_inside_polygon(polygon, grid_spacing=1):
    """
    Get the grid points inside a polygon
    :param polygon: the polygon (list of (x, y) tuples). e.g. [(0, 0), (2, 0), (3, 1), (2, 2), (0, 2), (-1, 1)]
    :return: list of (x, y) tuples inside the polygon
    """
    # Define a polygon with more than 4 points (example: hexagon)
    polygon = ShapelyPolygon(polygon)

    # Get the bounding box of the polygon
    min_x, min_y, max_x, max_y = polygon.bounds

    # Generate grid points
    grid_points = []
    x = min_x
    while x <= max_x:
        y = min_y
        while y <= max_y:
            point = ShapelyPoint(x, y)
            # Include points inside or on the edge of the polygon
            if polygon.contains(point) or polygon.touches(point):
                grid_points.append((x, y))
            y += grid_spacing
        x += grid_spacing

    return grid_points

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
    # 2. q1 * q2^-1 (q2 inverse/conjugation)

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
    if not isinstance(pose1, Pose):
        raise ValueError("pose1 must be a Pose")
    
    if not isinstance(pose2, Pose):
        raise ValueError("pose2 must be a Pose")

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
    # rospy.logerr(f'origin: {origin}, resolution: {resolution}')

    if resolution == 0:
        return 0, 0
    grid_x = int((x - origin.x) / resolution)
    grid_y = int((y - origin.y) / resolution)
    return grid_x, grid_y

def grid_to_index(grid_x, grid_y, map_info: MapMetaData):
    # rotasikan berdasarkan MapMetaData origin orientation
    return (map_info.width * grid_x) + -1*grid_y

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