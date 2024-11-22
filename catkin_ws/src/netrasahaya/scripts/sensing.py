import math
import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
import message_filters
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
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

def rotate_pose(pose, rx, ry, rz):
    position = [pose.position.x, pose.position.y, pose.position.z]
    orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rot_matrix = tf_conversions.transformations.euler_matrix(rx, ry, rz, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_position = quat_rotate(q_rot, position)
    new_orientation = tf_conversions.transformations.quaternion_multiply(orientation, q_rot)

    new_pose = Pose()
    new_pose.position.x = new_position[0]
    new_pose.position.y = new_position[1]
    new_pose.position.z = new_position[2]
    new_pose.orientation.x = new_orientation[0]
    new_pose.orientation.y = new_orientation[1]
    new_pose.orientation.z = new_orientation[2]
    new_pose.orientation.w = new_orientation[3]

    return new_pose

def local_to_global(pose, x, y, z):
    yaw = get_euler_pose(pose)[5]

    new_pose = Pose()
    new_pose.position.x = x
    new_pose.position.y = y
    new_pose.position.z = z

    new_pose = rotate_pose(new_pose, 0, 0, yaw)

    rotated_pose = Pose()
    rotated_pose.position.x = pose.position.x + new_pose.position.x
    rotated_pose.position.y = pose.position.y + new_pose.position.y
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

def raycast_occupancy_check(pose, occupancy_grid, distance=1.0):
    
    map_data = occupancygrid_to_numpy(occupancy_grid)
    
    # Convert start and end points to grid indices
    x, y, z, roll, pitch, yaw = get_euler_pose(pose)

    start_x, start_y = world_to_grid(x, y, occupancy_grid.info)
    half_pose = local_to_global(pose, distance/2, 0, 0)
    end_pose = local_to_global(pose, distance, 0, 0)
    half_x = half_pose.position.x
    half_y = half_pose.position.y
    end_x = end_pose.position.x
    end_y = end_pose.position.y
    half_grid_x, half_grid_y = world_to_grid(half_x, half_y, occupancy_grid.info)
    end_grid_x, end_grid_y = world_to_grid(end_x, end_y, occupancy_grid.info)

    # kiri_pose = PoseStamped()
    # kiri_pose.header.stamp = rospy.Time.now()
    # kiri_pose.header.frame_id = 'map'
    # kiri_pose.pose.position.x = end_x
    # kiri_pose.pose.position.y = end_y
    # kiri_pose.pose.position.z = pose.position.z

    # kiri_pose_pub.publish(kiri_pose)
    
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
    
    return severity

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
    depan_kiri_pub = rospy.Publisher('/depan_kiri', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kanan_pub = rospy.Publisher('/depan_kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m

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

            # # kiri_pose.pose = rotate_pose(kiri_pose.pose, 0, 0, math.pi/2)

            # kiri_pose_pub.publish(kiri_pose)

            kiri_severity = raycast_occupancy_check(odom.pose.pose, occupancy)

            rospy.loginfo(f'Kiri severity: {kiri_severity}')

            rate.sleep()
        except rospy.ROSInterruptException:
            break

    exit(0)