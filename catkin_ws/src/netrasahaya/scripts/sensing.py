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

    # Get 5 points of polygon to be checked
    point1 = Point(x=0, y=0, z=0)
    point2 = utils.rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta + theta_range)
    point3 = utils.rotate_point(Point(x=distance, y=0, z=0), 0, 0, theta - theta_range)
    halfpoint2 = utils.rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta + theta_range)
    halfpoint3 = utils.rotate_point(Point(x=distance/2, y=0, z=0), 0, 0, theta - theta_range)

    # Local to global
    point1_pose = utils.local_to_global(pose, point1)
    point2_pose = utils.local_to_global(pose, point2)
    point3_pose = utils.local_to_global(pose, point3)
    halfpoint2_pose = utils.local_to_global(pose, halfpoint2)
    halfpoint3_pose = utils.local_to_global(pose, halfpoint3)

    # Point to grid
    point1_grid_x, point1_grid_y = utils.world_to_grid(point1_pose.position.x, point1_pose.position.y, occupancy_grid.info)
    point2_grid_x, point2_grid_y = utils.world_to_grid(point2_pose.position.x, point2_pose.position.y, occupancy_grid.info)
    point3_grid_x, point3_grid_y = utils.world_to_grid(point3_pose.position.x, point3_pose.position.y, occupancy_grid.info)
    halfpoint2_grid_x, halfpoint2_grid_y = utils.world_to_grid(halfpoint2_pose.position.x, halfpoint2_pose.position.y, occupancy_grid.info)
    halfpoint3_grid_x, halfpoint3_grid_y = utils.world_to_grid(halfpoint3_pose.position.x, halfpoint3_pose.position.y, occupancy_grid.info)

    # VISUALIZATION
    polygon = PolygonStamped()
    polygon.header.frame_id = 'map'
    polygon.header.stamp = rospy.Time.now()
    polygon.polygon = Polygon([Point32(x=point1_pose.position.x, y=point1_pose.position.y, z=pose.position.z), Point32(x=point2_pose.position.x, y=point2_pose.position.y, z=pose.position.z), Point32(x=point3_pose.position.x, y=point3_pose.position.y, z=pose.position.z), Point32(x=halfpoint3_pose.position.x, y=halfpoint3_pose.position.y, z=pose.position.z), Point32(x=halfpoint2_pose.position.x, y=halfpoint2_pose.position.y, z=pose.position.z), Point32(x=halfpoint3_pose.position.x, y=halfpoint3_pose.position.y, z=pose.position.z)])
    # VISUALIZATION
    
    # Bresenham's line algorithm
    # line_start_half = bresenham(start_x, start_y, half_grid_x, half_grid_y)
    # line_half_end = bresenham(half_grid_x, half_grid_y, end_grid_x, end_grid_y)
    near_points = utils.bresenham_polygon_with_fill([(point1_grid_x, point1_grid_y), (halfpoint2_grid_x, halfpoint2_grid_y), (halfpoint3_grid_x, halfpoint3_grid_y)])
    far_points = utils.bresenham_polygon_with_fill([(halfpoint2_grid_x, halfpoint2_grid_y), (point2_grid_x, point2_grid_y), (point3_grid_x, point3_grid_y), (halfpoint3_grid_x, halfpoint3_grid_y)])

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