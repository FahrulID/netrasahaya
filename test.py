from collections import deque
from matplotlib import pyplot as plt
import numpy as np
import math
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf_conversions
from matplotlib.path import Path
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry.polygon import Polygon as ShapelyPolygon

map = np.loadtxt("/home/fahrul/netrasahaya/map.npy", dtype=int)

kiri = 67.5 * math.pi/180
kanan = -kiri
kiri_depan = 22.5 * math.pi/180
kanan_depan = -kiri_depan
theta = [kiri, kiri_depan, kanan_depan, kanan]
theta_range = math.pi/4

pose = Pose()
pose.position.x = 7.871008396148682
pose.position.y = 1.4590764045715332
pose.position.z = 0.860668957233429

# euler angle to quaternion
# q = tf_conversions.transformations.quaternion_from_euler(0, 0, math.pi/2)
# pose.orientation.x = q[0]
# pose.orientation.y = q[1]
# pose.orientation.z = q[2]
# pose.orientation.w = q[3]

pose.orientation.x = 0.009308643639087677
pose.orientation.y = -0.021675776690244675
pose.orientation.z = -0.5103920698165894
pose.orientation.w = 0.8596182465553284

max_distance = 1.5

info = MapMetaData()
info.resolution = 0.05
info.origin.position.x = -1.850000
info.origin.position.y = 3.050000
info.origin.orientation.x = 0.0
info.origin.orientation.y = 0.0
info.origin.orientation.z = 0.7071066498756409
info.origin.orientation.w = -0.70710688829422
info.width = 136
info.height = 265

def get_grid_section(cell_coords: list):
    """
    Extract and format a section of occupancy grid based on given cell coordinates
    
    Args:
        msg: OccupancyGrid message
        cell_coords: List of tuples containing (x,y) cell coordinates
    
    Returns:
        numpy array with specified section, other cells marked as 100
    """

    # Convert original grid to numpy array
    original_grid = map

    # Create new array filled with 100 (occupied)
    # formatted_grid = np.full((info.height, info.width), 100, dtype=np.int8)

    # make an array as big as original_grid, but every cell is 100
    formatted_grid = np.full(original_grid.shape, 100, dtype=np.int8)

    # Copy values from original grid for specified coordinates
    for x, y in cell_coords:
        x = int(x)
        y = int(y)
        formatted_grid[y, x] = original_grid[y, x]

    return formatted_grid

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

def bresenham_line(start_point, end_point):
    """Get all points in line using Bresenham's algorithm with numpy
    Args:
        start_point: tuple (x1, y1)
        end_point: tuple (x2, y2)
    Returns:
        np.array: Array of points [(x,y), ...]
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

def occupancygrid_to_numpy(msg: OccupancyGrid):
    """
    Convert an OccupancyGrid message to a numpy array
    """
    if not isinstance(msg, OccupancyGrid):
        raise ValueError("msg must be an OccupancyGrid")
    
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    return np.ma.array(data, mask=data==-1, fill_value=-1)

def get_occupancy_segment_with_points(occupancy: OccupancyGrid, points: list, target_points: list):
    max_x = max([point[0] for point in points])
    max_y = max([point[1] for point in points])
    min_x = min([point[0] for point in points])
    min_y = min([point[1] for point in points])
    map_data = occupancygrid_to_numpy(occupancy)

    # create numpy array with the size of the bounding box
    grid = np.all(100, (max_y - min_y + 1, max_x - min_x + 1), dtype=np.int8)

    new_target_points = []

    for point in points:
        x, y = point
        grid[y - min_y, x - min_x] = map_data[y, x]
        new_target_points.append((x - min_x, y - min_y))

    return grid, new_target_points

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
    vertices.append(start_point)

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

def is_path_possible(grid, start_points, end_points) -> bool:
    """
    Utilizing BFS to find if there is a path from any cell in the first row to any cell in the last row
    :param grid: the grid
    :param start_points: the starting points
    :param end_points: the ending points
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
        x, y = queue.popleft()

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

            is_occupied = grid[nx, ny] == 1
            if is_occupied:
                continue

            queue.append((nx, ny))
            visited.add((nx, ny))

    return False  # No path found

def check_passability(start, end_points, map_data):
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
                (nx, ny) not in visited and map_data[ny][nx] < 50):
                queue.append((nx, ny))
                visited.add((nx, ny))
    
    return False

for t in theta:

    start, end, vertices, edge_points, inside_points = get_polygon_points(pose, t, theta_range, max_distance, info)
    
    # Find nearest obstacle
    # distance = find_nearest_obstacle(pose, points, map_data, occupancy.info)
    
    # Check passability
    # segment_map = get_grid_section(inside_points)
    print(world_to_grid(info.origin.position.x + 2, info.origin.position.y + 1, info))
    # print(grid_to_world(start[0], start[1], info))
    # is_passable = check_passability(start, end, segment_map)

    # print(is_passable)

