import math
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, PolygonStamped, Polygon, Point32
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int32
import message_filters
import utils.utils as utils

class SensingDisabled(Exception):
    pass

def raycast_occupancy_check(pose, occupancy_grid, distance=1.0, theta=0.0, theta_range=0.0):
    
    map_data = utils.occupancygrid_to_numpy(occupancy_grid)
    
    near_polygon, far_polygon = utils.get_polygon_segment_from_pose(pose, distance, theta, theta_range)
    # print("far_polygon", far_polygon)
    # print("pose", pose)
    # print("distance", distance)
    # print("theta", theta)
    near_points = utils.get_grid_points_inside_polygon(near_polygon, occupancy_grid.info.resolution)
    far_points = utils.get_grid_points_inside_polygon(far_polygon, occupancy_grid.info.resolution)

    near_points_grid = []

    for x, y in near_points:
        # print("near x y", x, y)
        a, b = utils.world_to_grid(x, y, occupancy_grid.info)
        rotated_point = utils.rotate_point(Point(x=a, y=b, z=0), 0, 0, math.pi/2)
        a = int(round(rotated_point.x))
        b = int(round(rotated_point.y))
        # print("near a b", a, b)
        near_points_grid.append((a, b))

    far_points_grid = []

    for x, y in far_points:
        # print("far x y", x, y)
        a, b = utils.world_to_grid(x, y, occupancy_grid.info)
        rotated_point = utils.rotate_point(Point(x=a, y=b, z=0), 0, 0, math.pi/2)
        a = int(round(rotated_point.x))
        b = int(round(rotated_point.y))
        # print("far a b", a, b)
        far_points_grid.append((a, b))

    severity = 0

    if(utils.check_if_points_in_grid_map_are_occupied(near_points_grid, map_data, 10)):
        severity = 1
    elif(utils.check_if_points_in_grid_map_are_occupied(far_points_grid, map_data, 10)):
        severity = 2

    # VISUALIZATION
    polygon = PolygonStamped()
    polygon.header.frame_id = 't265_odom_frame'
    polygon.header.stamp = rospy.Time.now()
    points = []
    for x, y in near_polygon:
        points.append(Point32(x=x, y=y, z=0))
    points.append(Point32(x=near_polygon[0][0], y=near_polygon[0][1], z=0))
    for x, y in far_polygon:
        points.append(Point32(x=x, y=y, z=0))
    points.append(Point32(x=far_polygon[0][0], y=far_polygon[0][1], z=0))

    severity_pose = pose

    if severity == 2:
        severity_pose = utils.local_to_global(pose, utils.rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta + (theta_range / 2)))
    elif severity == 1:
        severity_pose = utils.local_to_global(pose, utils.rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta + (theta_range / 2)))
    points.append(Point32(x=severity_pose.position.x, y=severity_pose.position.y, z=0))

    polygon.polygon = Polygon(points)
    # VISUALIZATION
    
    return severity, polygon

odom = Odometry()
occupancy = OccupancyGrid()

def callback(odom_msg, occupancy_msg):
    global odom, occupancy
    odom = odom_msg
    occupancy = occupancy_msg

if __name__ == '__main__':
    rospy.init_node('sensing')

    odometry_sub = message_filters.Subscriber('/t265/odom/sample', Odometry)
    occupancy_sub = message_filters.Subscriber('/occupancy', OccupancyGrid)

    ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, occupancy_sub], 10, 0.1)
    ts.registerCallback(callback)

    kiri_pub = rospy.Publisher('/kiri', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    kiri_polygon_pub = rospy.Publisher('/kiri_polygon', PolygonStamped, queue_size=10)
    kanan_pub = rospy.Publisher('/kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    kanan_polygon_pub = rospy.Publisher('/kanan_polygon', PolygonStamped, queue_size=10)
    depan_kiri_pub = rospy.Publisher('/depan_kiri', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kiri_polygon_pub = rospy.Publisher('/depan_kiri_polygon', PolygonStamped, queue_size=10)
    depan_kanan_pub = rospy.Publisher('/depan_kanan', Int32, queue_size=10) # 0 = tidak ada, 1 = ada pada jarak 0.5m, 2 = ada pada jarak 1m
    depan_kanan_polygon_pub = rospy.Publisher('/depan_kanan_polygon', PolygonStamped, queue_size=10)

    visualization_enabled = rospy.get_param("/sensing/visualize", False)

    rospy.logerr(f'Visualization enabled: {visualization_enabled}')

    rate = rospy.Rate(30.0)
    
    while not rospy.is_shutdown():
        try:
            enabled = rospy.get_param("/mode", "none") == "sensing"

            if not enabled:
                raise SensingDisabled('Sensing is disabled')
            
            if odom == Odometry() or occupancy == OccupancyGrid():
                raise SensingDisabled('No data received')
            
            theta1 = math.pi*1/2 - math.pi*1/8 # 90 - 22.5 = 67.5 degree
            theta2 = math.pi*1/2 - math.pi*3/8 # 90 - 67.5 = 22.5 degree
            theta_range = math.pi*1/8

            kiri_severity, kiri_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, theta1, theta_range)
            kiri_pub.publish(kiri_severity)

            kanan_severity, kanan_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, -theta1, theta_range)
            kanan_pub.publish(kanan_severity)

            depan_kiri_severity, depan_kiri_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, theta2, theta_range)
            depan_kiri_pub.publish(depan_kiri_severity)

            depan_kanan_severity, depan_kanan_polygon = raycast_occupancy_check(odom.pose.pose, occupancy, 1, -theta2, theta_range)
            depan_kanan_pub.publish(depan_kanan_severity)

            if visualization_enabled:
                kiri_polygon_pub.publish(kiri_polygon)
                kanan_polygon_pub.publish(kanan_polygon)
                depan_kiri_polygon_pub.publish(depan_kiri_polygon)
                depan_kanan_polygon_pub.publish(depan_kanan_polygon)
                # rospy.logerr(f'Kiri: {kiri_severity}, Depan Kiri: {depan_kiri_severity}, Depan Kanan: {depan_kanan_severity}, Kanan: {kanan_severity}')
        except SensingDisabled as e:
            rospy.logerr_throttle(1, e)
            pass   
        except rospy.ROSInterruptException:
            break
        except Exception as e:
            rospy.logerr_throttle(e)
            break
        
        rate.sleep()

    exit(0)