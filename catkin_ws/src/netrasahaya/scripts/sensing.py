import math
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import message_filters
from std_msgs.msg import Int32
import numpy as np

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

def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)

    return np.ma.array(data, mask=data==-1, fill_value=-1)

def raycast_occupancy_check(pose, occupancy_grid, distance=1.0, theta=0.0):
    
    map_data = occupancygrid_to_numpy(occupancy_grid)
    
    # Convert start and end points to grid indices
    x, y, z, roll, pitch, yaw = get_euler_pose(pose)

    # Rotate the ray by theta
    full_point = Point(x=distance, y=0, z=0)
    rotated_full_point = rotate_point(full_point, 0, 0, theta)

    half_point = Point(x=distance/2, y=0, z=0)
    rotated_half_point = rotate_point(half_point, 0, 0, theta)

    start_x, start_y = world_to_grid(x, y, occupancy_grid.info)

    half_pose = local_to_global(pose, rotated_half_point)

    end_pose = local_to_global(pose, rotated_full_point)

    half_x = half_pose.position.x
    half_y = half_pose.position.y
    end_x = end_pose.position.x
    end_y = end_pose.position.y

    half_grid_x, half_grid_y = world_to_grid(half_x, half_y, occupancy_grid.info)
    end_grid_x, end_grid_y = world_to_grid(end_x, end_y, occupancy_grid.info)

    # DEBUGGING
    pose_to_check = PoseStamped()
    pose_to_check.header.stamp = rospy.Time.now()
    pose_to_check.header.frame_id = 'map'
    pose_to_check.pose.position.x = end_x
    pose_to_check.pose.position.y = end_y
    pose_to_check.pose.position.z = pose.position.z
    pose_to_check.pose.orientation = pose.orientation
    # DEBUGGING
    
    # Bresenham's line algorithm
    line_start_half = bresenham(start_x, start_y, half_grid_x, half_grid_y)
    line_half_end = bresenham(half_grid_x, half_grid_y, end_grid_x, end_grid_y)

    # Severity of the obstacle, 0 means no obstacle, 1 means obstacle at 0.5m, 2 means obstacle at 1m
    severity = 0
    
    # Check each cell in the ray
    for line_x, line_y in line_half_end:
        # check if the cell is within the map
        if line_x < 0 or line_y < 0 or line_x >= map_data.shape[1] or line_y >= map_data.shape[0]:
            continue

        # Check if the cell is occupied
        if map_data[line_y, line_x] > 0:
            severity = 2

    for line_x, line_y in line_start_half:
        # check if the cell is within the map
        if line_x < 0 or line_y < 0 or line_x >= map_data.shape[1] or line_y >= map_data.shape[0]:
            continue

        # Check if the cell is occupied
        if map_data[line_y, line_x] > 0:
            severity = 1
    
    return severity, pose_to_check

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
    kiri_pose_pub = rospy.Publisher('/kiri_pose', PoseStamped, queue_size=10)
    kanan_pub = rospy.Publisher('/kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    kanan_pose_pub = rospy.Publisher('/kanan_pose', PoseStamped, queue_size=10)
    depan_kiri_pub = rospy.Publisher('/depan_kiri', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kiri_pose_pub = rospy.Publisher('/depan_kiri_pose', PoseStamped, queue_size=10)
    depan_kanan_pub = rospy.Publisher('/depan_kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kanan_pose_pub = rospy.Publisher('/depan_kanan_pose', PoseStamped, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(30.0)
    
    while not rospy.is_shutdown():
        try:
            # x, y, z, roll, pitch, yaw = get_euler_pose(odom.pose.pose)

            # kiri_pose = PoseStamped()
            # kiri_pose.header.stamp = rospy.Time.now()
            # kiri_pose.header.frame_id = 'map'
            # kiri_pose.pose = local_to_global(odom.pose.pose, 1, 0, 0)

            # # kiri_pose.pose = rotate_point(kiri_pose.pose, 0, 0, math.pi/2)

            # kiri_pose_pub.publish(kiri_pose)

            kiri_severity, kiri_pose = raycast_occupancy_check(odom.pose.pose, occupancy, 1, math.pi*7/16)
            kiri_pub.publish(kiri_severity)
            kiri_pose_pub.publish(kiri_pose)

            kanan_severity, kanan_pose = raycast_occupancy_check(odom.pose.pose, occupancy, 1, -math.pi*7/16)
            kanan_pub.publish(kanan_severity)
            kanan_pose_pub.publish(kanan_pose)

            depan_kiri_severity, depan_kiri_pose = raycast_occupancy_check(odom.pose.pose, occupancy, 1, math.pi*2/16)
            depan_kiri_pub.publish(depan_kiri_severity)
            depan_kiri_pose_pub.publish(depan_kiri_pose)

            depan_kanan_severity, depan_kanan_pose = raycast_occupancy_check(odom.pose.pose, occupancy, 1, -math.pi*2/16)
            depan_kanan_pub.publish(depan_kanan_severity)
            depan_kanan_pose_pub.publish(depan_kanan_pose)

            rospy.loginfo(f'Kiri: {kiri_severity}, Kanan: {kanan_severity}, Depan Kiri: {depan_kiri_severity}, Depan Kanan: {depan_kanan_severity}')

            rate.sleep()
        except rospy.ROSInterruptException:
            break

    exit(0)