import math
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped, Polygon, Point32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import message_filters
from std_msgs.msg import Int32
import numpy as np

class SensingDisabled(Exception):
    pass

def quat_rotate(rotation, vector):
    """
    Rotate a vector according to a quaternion. Equivalent to the C++ method tf::quatRotate
    :param rotation: the rotation
    :param vector: the vector to rotate
    :return: the rotated vector
    """

    def quat_mult_point(q, w):
        return (q[3] * w[0] + q[1] * w[2] - q[2] * w[1],
                q[3] * w[1] + q[2] * w[0] - q[0] * w[2],
                q[3] * w[2] + q[0] * w[1] - q[1] * w[0],
                -q[0] * w[0] - q[1] * w[1] - q[2] * w[2])

    q = quat_mult_point(rotation, vector)
    q = tf_conversions.transformations.quaternion_multiply(q, tf_conversions.transformations.quaternion_inverse(rotation))
    return [q[0], q[1], q[2]]

def rotate_point(point: Point, rx: float, ry: float, rz: float) -> Point:
    if not isinstance(point, Point):
        raise ValueError("point must be a Point")

    position = [point.x, point.y, point.z]
    rot_matrix = tf_conversions.transformations.euler_matrix(rx, ry, rz, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_position = quat_rotate(q_rot, position)

    return Point(x=new_position[0], y=new_position[1], z=new_position[2])

def local_to_global(pose: Pose, point: Point) -> Pose:
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

def world_to_grid(x, y, map_info):
    origin = map_info.origin.position
    resolution = map_info.resolution

    if resolution == 0:
        return 0, 0

    grid_x = int((x - origin.x) / resolution)
    grid_y = int((y - origin.y) / resolution)
    return grid_x, grid_y

def get_euler_pose(pose):
    position = pose.position
    orientation = pose.orientation
    euler = tf_conversions.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    roll, pitch, yaw = euler
    return position.x, position.y, position.z, roll, pitch, yaw

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

def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    return np.ma.array(data, mask=data==-1, fill_value=-1)

def raycast_occupancy_check(pose, occupancy_grid, distance=1.0, theta=0.0, theta_range=0.0):
    
    map_data = occupancygrid_to_numpy(occupancy_grid)
    
    # Convert start and end points to grid indices
    x, y, z, roll, pitch, yaw = get_euler_pose(pose)

    # Get 5 points of polygon to be checked
    point1 = Point(x=0, y=0, z=0)
    point2 = rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta + theta_range)
    point3 = rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta - theta_range)
    halfpoint2 = rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta + theta_range)
    halfpoint3 = rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta - theta_range)

    # Local to global
    point1_pose = local_to_global(pose, point1)
    point2_pose = local_to_global(pose, point2)
    point3_pose = local_to_global(pose, point3)
    halfpoint2_pose = local_to_global(pose, halfpoint2)
    halfpoint3_pose = local_to_global(pose, halfpoint3)

    # Point to grid
    point1_grid_x, point1_grid_y = world_to_grid(point1_pose.position.x, point1_pose.position.y, occupancy_grid.info)
    point2_grid_x, point2_grid_y = world_to_grid(point2_pose.position.x, point2_pose.position.y, occupancy_grid.info)
    point3_grid_x, point3_grid_y = world_to_grid(point3_pose.position.x, point3_pose.position.y, occupancy_grid.info)
    halfpoint2_grid_x, halfpoint2_grid_y = world_to_grid(halfpoint2_pose.position.x, halfpoint2_pose.position.y, occupancy_grid.info)
    halfpoint3_grid_x, halfpoint3_grid_y = world_to_grid(halfpoint3_pose.position.x, halfpoint3_pose.position.y, occupancy_grid.info)

    # VISUALIZATION
    polygon = PolygonStamped()
    polygon.header.frame_id = 'map'
    polygon.header.stamp = rospy.Time.now()
    polygon.polygon = Polygon([Point32(x=point1_pose.position.x, y=point1_pose.position.y, z=pose.position.z), Point32(x=point2_pose.position.x, y=point2_pose.position.y, z=pose.position.z), Point32(x=point3_pose.position.x, y=point3_pose.position.y, z=pose.position.z), Point32(x=halfpoint3_pose.position.x, y=halfpoint3_pose.position.y, z=pose.position.z), Point32(x=halfpoint2_pose.position.x, y=halfpoint2_pose.position.y, z=pose.position.z), Point32(x=halfpoint3_pose.position.x, y=halfpoint3_pose.position.y, z=pose.position.z)])
    # VISUALIZATION
    
    # Bresenham's line algorithm
    # line_start_half = bresenham(start_x, start_y, half_grid_x, half_grid_y)
    # line_half_end = bresenham(half_grid_x, half_grid_y, end_grid_x, end_grid_y)
    near_points = bresenham_polygon_with_fill([(point1_grid_x, point1_grid_y), (halfpoint2_grid_x, halfpoint2_grid_y), (halfpoint3_grid_x, halfpoint3_grid_y)])
    far_points = bresenham_polygon_with_fill([(halfpoint2_grid_x, halfpoint2_grid_y), (point2_grid_x, point2_grid_y), (point3_grid_x, point3_grid_y), (halfpoint3_grid_x, halfpoint3_grid_y)])

    # Severity of the obstacle, 0 means no obstacle, 1 means obstacle at 0.5m, 2 means obstacle at 1m
    severity = 0
    
    # Check each cell in the ray
    for line_x, line_y in far_points:
        # check if the cell is within the map
        if line_x < 0 or line_y < 0 or line_x >= map_data.shape[1] or line_y >= map_data.shape[0]:
            continue

        # Check if the cell is occupied
        if map_data[line_y, line_x] > 0:
            severity = 2
            break

    for line_x, line_y in near_points:
        # check if the cell is within the map
        if line_x < 0 or line_y < 0 or line_x >= map_data.shape[1] or line_y >= map_data.shape[0]:
            continue

        # Check if the cell is occupied
        if map_data[line_y, line_x] > 0:
            severity = 1
            break
    
    return severity, polygon

odom = Odometry()
occupancy = OccupancyGrid()

def callback(odom_msg, occupancy_msg):
    global odom, occupancy
    odom = odom_msg
    occupancy = occupancy_msg

if __name__ == '__main__':
    rospy.init_node('sensing')

    odometry_sub = message_filters.Subscriber('/odom', Odometry)
    occupancy_sub = message_filters.Subscriber('/occupancy/map', OccupancyGrid)

    ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, occupancy_sub], 10, 0.1)
    ts.registerCallback(callback)

    kiri_pub = rospy.Publisher('/kiri', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    kiri_polygon_pub = rospy.Publisher('/kiri_pose', PolygonStamped, queue_size=10)
    kanan_pub = rospy.Publisher('/kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    kanan_polygon_pub = rospy.Publisher('/kanan_pose', PolygonStamped, queue_size=10)
    depan_kiri_pub = rospy.Publisher('/depan_kiri', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kiri_polygon_pub = rospy.Publisher('/depan_kiri_pose', PolygonStamped, queue_size=10)
    depan_kanan_pub = rospy.Publisher('/depan_kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kanan_polygon_pub = rospy.Publisher('/depan_kanan_pose', PolygonStamped, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(30.0)
    
    while not rospy.is_shutdown():
        try:
            enabled = rospy.get_param('~enabled', False)

            if not enabled:
                rospy.loginfo(enabled)
                raise SensingDisabled('Sensing is disabled')
            
            theta1 = math.pi*1/2 - math.pi*1/8 # 90 - 22.5 = 67.5 degree
            theta2 = math.pi*1/2 - math.pi*3/8 # 90 - 67.5 = 22.5 degree
            theta_range = math.pi*1/8

            kiri_severity, kiri_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, theta1, theta_range)
            kiri_pub.publish(kiri_severity)
            kiri_polygon_pub.publish(kiri_polygon)

            kanan_severity, kanan_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, -theta1, theta_range)
            kanan_pub.publish(kanan_severity)
            kanan_polygon_pub.publish(kanan_polygon)

            depan_kiri_severity, depan_kiri_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, theta2, theta_range)
            depan_kiri_pub.publish(depan_kiri_severity)
            depan_kiri_polygon_pub.publish(depan_kiri_polygon)

            depan_kanan_severity, depan_kanan_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, -theta2, theta_range)
            depan_kanan_pub.publish(depan_kanan_severity)
            depan_kanan_polygon_pub.publish(depan_kanan_polygon)

            rospy.loginfo(f'Kiri: {kiri_severity}, Depan Kiri: {depan_kiri_severity}, Depan Kanan: {depan_kanan_severity}, Kanan: {kanan_severity}')
        except SensingDisabled:
            rospy.loginfo('Sensing is disabled')        
        except rospy.ROSInterruptException:
            break
        except Exception as e:
            rospy.logerr(e)
            break
        
        rate.sleep()

    exit(0)