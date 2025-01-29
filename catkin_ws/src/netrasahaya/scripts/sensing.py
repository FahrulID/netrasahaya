import math
import rospy
import numpy as np
from enum import Enum
from collections import deque
from geometry_msgs.msg import Pose, Point, PolygonStamped, Point32
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Int32, Float32
import message_filters
import utils.utils as utils
from sensor_msgs.msg import PointCloud

class OccupancyState(Enum):
    FREE = 0
    NAVIGABLE = 1
    BLOCKED = 2

class Segment:
    def __init__(self, name, theta, theta_range):
        self.name = name
        self.theta = theta
        self.theta_range = theta_range
        self.state_pub = rospy.Publisher(f'/{name}/state', Int32, queue_size=10)
        self.distance_pub = rospy.Publisher(f'/{name}/distance', Float32, queue_size=10)
        self.polygon_pub = rospy.Publisher(f'/{name}/polygon', PolygonStamped, queue_size=10)
        self.pointcloud_pub = rospy.Publisher(f'/{name}/pointcloud', PointCloud, queue_size=10)

def create_polygon(points, frame_id):
    msg = PolygonStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.polygon.points = [Point32(x=x, y=y, z=0) for x, y in points]
    return msg

def create_pointcloud(points, frame_id):
    msg = PointCloud()
    msg.header.frame_id = frame_id
    msg.header.stamp = rospy.Time.now()
    msg.points = [Point(x=x, y=y, z=0) for x, y in points]
    return msg

def find_nearest_obstacle(center_pose, points, map_data, occupancy_info):
    min_distance = float('inf')
    center_x, center_y = center_pose.position.x, center_pose.position.y
    
    for x, y in points:
        if 0 <= y < map_data.shape[0] and 0 <= x < map_data.shape[1]:
            if map_data[y][x] > 50:  # Occupied threshold
                world_x, world_y = utils.grid_to_world(x, y, occupancy_info)
                dist = math.sqrt((world_x - center_x)**2 + (world_y - center_y)**2)
                min_distance = min(min_distance, dist)
    
    return min_distance if min_distance != float('inf') else -1

def process_segment(segment, pose, occupancy, max_distance=1.5):
    map_data = utils.occupancygrid_to_numpy(occupancy)
    
    # Create arc points for segment
    start, end, vertices, edge_points, inside_points = utils.get_polygon_points(pose, segment.theta, segment.theta_range, max_distance, occupancy.info)
    
    # Find nearest obstacle
    # distance = find_nearest_obstacle(pose, points, map_data, occupancy.info)
    
    # Check passability
    # print(f'Processing segment {segment.name}')
    segment_map, start_point, end_points = utils.get_grid_section(occupancy, inside_points, start, end)
    is_passable = utils.check_passability(start_point, end_points, segment_map)
    distance = utils.check_nearest_occupied(start_point, segment_map, occupancy.info)
    
    # Determine state
    if distance == -1:
        state = OccupancyState.FREE
    elif is_passable:
        state = OccupancyState.NAVIGABLE
    else:
        state = OccupancyState.BLOCKED

    # Create visualization
    points_viz = []
    for x, y in edge_points:
        points_viz.append(utils.grid_to_world(x, y, occupancy.info))
    polygon = create_polygon(points_viz, 't265_odom_frame')

    # Create pointcloud
    points_viz_cloud = []
    for x, y in inside_points:
        points_viz_cloud.append(utils.grid_to_world(x, y, occupancy.info))
    pointcloud = create_pointcloud(points_viz_cloud, 't265_odom_frame')

    return distance, state, polygon, pointcloud

class SensingNode:
    def __init__(self):
        rospy.init_node('sensing')
        
        # Define segments
        self.segments = [
            Segment('front_left',  (22.5 * math.pi/180), math.pi/4),
            Segment('front_right', -(22.5 * math.pi/180), math.pi/4),
            Segment('left',  (67.5 * math.pi/180), math.pi/4),
            Segment('right', -(67.5 * math.pi/180), math.pi/4)
        ]
        
        # Setup subscribers
        self.odom = None
        self.occupancy = None
        odometry_sub = message_filters.Subscriber('/t265/odom/sample', Odometry)
        occupancy_sub = message_filters.Subscriber('/occupancy', OccupancyGrid)
        ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, occupancy_sub], 10, 0.1)
        ts.registerCallback(self.callback)
        
        self.rate = rospy.Rate(30.0)
        self.visualization_enabled = rospy.get_param("/sensing/visualize", False)
    
    def callback(self, odom_msg, occupancy_msg):
        self.odom = odom_msg
        self.occupancy = occupancy_msg
    
    def run(self):
        while not rospy.is_shutdown():
            enabled = rospy.get_param("/mode", "none") == "sensing"
            
            if not enabled:
                self.rate.sleep()
                continue

            if self.odom and self.occupancy:
                for segment in self.segments:
                    distance, state, polygon, pointcloud = process_segment(
                        segment, self.odom.pose.pose, self.occupancy)
                    
                    segment.state_pub.publish(state.value)
                    segment.distance_pub.publish(distance)
                    
                    if self.visualization_enabled:
                        segment.polygon_pub.publish(polygon)
                        segment.pointcloud_pub.publish(pointcloud)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = SensingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass