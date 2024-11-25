#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import Twist, WrenchStamped, Pose, Point, Quaternion
import message_filters
import math
import utils.utils as utils

pose_offset = Pose()
offset_initialized = False

def callback(odometry, posewithcovariancestamped):
    # check for covariance for x, y, z, rx, ry, rz
    cov_x = posewithcovariancestamped.pose.covariance[0]
    cov_y = posewithcovariancestamped.pose.covariance[7]
    cov_z = posewithcovariancestamped.pose.covariance[14]
    cov_rx = posewithcovariancestamped.pose.covariance[21]
    cov_ry = posewithcovariancestamped.pose.covariance[28]
    cov_rz = posewithcovariancestamped.pose.covariance[35]

    if cov_x < 0.01 and cov_y < 0.01 and cov_z < 0.01 and cov_rx < 0.01 and cov_ry < 0.01 and cov_rz < 0.01:
        # check if pose_offset is not set
        if offset_initialized == False:
            offset_initialized = True

            rel_pose = utils.align_pose(odometry.pose.pose.position, posewithcovariancestamped.pose.pose.position)
            pose_offset.position.x = rel_pose.position.x
            pose_offset.position.y = rel_pose.position.y
            pose_offset.position.z = rel_pose.position.z
            pose_offset.orientation.x = rel_pose.orientation.x
            pose_offset.orientation.y = rel_pose.orientation.y
            pose_offset.orientation.z = rel_pose.orientation.z
            pose_offset.orientation.w = rel_pose.orientation.w

            rospy.loginfo("Pose offset set to: " + str(pose_offset))
            # rospy.loginfo("Pose offset set to: " + str(pose_offset))

def handle_t265_odom(msg, now = None):
    base_link = TransformStamped()
    base_link.header.stamp = rospy.Time.now() if now == None else now
    base_link.header.frame_id = "odom"
    base_link.child_frame_id = "base_link"

    position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    rot_matrix = tf_conversions.transformations.euler_matrix(0, 0, math.pi/2, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_position = utils.quat_rotate(q_rot, position)

    base_link.transform.translation.x = new_position[0]
    base_link.transform.translation.y = new_position[1]
    base_link.transform.translation.z = new_position[2]
    
    new_orientation = tf_conversions.transformations.quaternion_multiply(orientation, q_rot)

    base_link.transform.rotation.x = new_orientation[0]
    base_link.transform.rotation.y = new_orientation[1]
    base_link.transform.rotation.z = new_orientation[2]
    base_link.transform.rotation.w = new_orientation[3]

    br.sendTransform(base_link)

    new_msg = msg
    new_msg.header.stamp = rospy.Time.now()
    new_msg.header.frame_id = "odom"
    new_msg.child_frame_id = "base_link"
    new_msg.pose.pose.position.x = new_position[0]
    new_msg.pose.pose.position.y = new_position[1]
    new_msg.pose.pose.position.z = new_position[2]
    new_msg.pose.pose.orientation.x = new_orientation[0]
    new_msg.pose.pose.orientation.y = new_orientation[1]
    new_msg.pose.pose.orientation.z = new_orientation[2]
    new_msg.pose.pose.orientation.w = new_orientation[3]
    odometry_pub.publish(new_msg)

def handle_d400_points(msg, now = None):
    camera_link = TransformStamped()
    camera_link.header.stamp = rospy.Time.now() if now == None else now
    camera_link.header.frame_id = "base_link"
    camera_link.child_frame_id = "camera_link"

    translation = [0, 0, 0]
    orientation = [0, 0, 0, 1]
    # rot_matrix = tf_conversions.transformations.euler_matrix(0, 0, 0)
    rot_matrix = tf_conversions.transformations.euler_matrix(0, math.pi/2, -math.pi/2, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_translation = utils.quat_rotate(q_rot, translation)

    camera_link.transform.translation.x = new_translation[0]
    camera_link.transform.translation.y = new_translation[1]
    camera_link.transform.translation.z = new_translation[2]
    
    new_orientation = tf_conversions.transformations.quaternion_multiply(orientation, q_rot)

    camera_link.transform.rotation.x = new_orientation[0]
    camera_link.transform.rotation.y = new_orientation[1]
    camera_link.transform.rotation.z = new_orientation[2]
    camera_link.transform.rotation.w = new_orientation[3]

    br.sendTransform(camera_link)

    new_msg = msg
    new_msg.header.stamp = rospy.Time.now()
    new_msg.header.frame_id = "camera_link"
    point_cloud_pub.publish(new_msg)

def handle_occupancy(msg):
    new_map = OccupancyGrid()
    new_map.header.stamp = rospy.Time.now()
    new_map.header.frame_id = "map"

    origin_position = [msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z]
    origin_orientation = [msg.info.origin.orientation.x, msg.info.origin.orientation.y, msg.info.origin.orientation.z, msg.info.origin.orientation.w]

    rot_matrix = tf_conversions.transformations.euler_matrix(0, 0, math.pi/2, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_origin_position = utils.quat_rotate(q_rot, origin_position)
    new_origin_orientation = tf_conversions.transformations.quaternion_multiply(origin_orientation, q_rot)
    
    new_map.info.map_load_time = msg.info.map_load_time
    new_map.info.resolution = msg.info.resolution
    new_map.info.width = msg.info.width
    new_map.info.height = msg.info.height
    new_map.info.origin.position.x = new_origin_position[0]
    new_map.info.origin.position.y = new_origin_position[1]
    new_map.info.origin.position.z = new_origin_position[2]
    new_map.info.origin.orientation.x = new_origin_orientation[0]
    new_map.info.origin.orientation.y = new_origin_orientation[1]
    new_map.info.origin.orientation.z = new_origin_orientation[2]
    new_map.info.origin.orientation.w = new_origin_orientation[3]
    new_map.data = msg.data

    map_pub.publish(new_map)

def handle_twist(msg):
    wrench = WrenchStamped()
    wrench.header.stamp = rospy.Time.now()
    wrench.header.frame_id = "base_link"
    
    wrench.wrench.force.x = msg.linear.x * 10
    wrench.wrench.force.y = msg.linear.y * 10
    wrench.wrench.force.z = msg.linear.z * 10

    wrench.wrench.torque.x = msg.angular.x * 10
    wrench.wrench.torque.y = msg.angular.y * 10
    wrench.wrench.torque.z = msg.angular.z * 10

    twist_pub.publish(wrench)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    br = tf2_ros.TransformBroadcaster()

    odometry_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    point_cloud_pub = rospy.Publisher('/depth', PointCloud2, queue_size=10)
    map_pub = rospy.Publisher('/occupancy/map', OccupancyGrid, queue_size=10)
    twist_pub = rospy.Publisher('/cmd_vel/stamped', WrenchStamped, queue_size=10)

    rospy.Subscriber('/t265/odom/sample', Odometry, handle_t265_odom)
    rospy.Subscriber('/d400/depth/color/points', PointCloud2, handle_d400_points)
    rospy.Subscriber('/occupancy', OccupancyGrid, handle_occupancy)
    rospy.Subscriber('/cmd_vel', Twist, handle_twist)

    odometry_sub = message_filters.Subscriber('/t265/odom/sample', Odometry)
    localization_sub = message_filters.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped)

    ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, localization_sub], 10, 0.1)
    ts.registerCallback(callback)

    rospy.spin()
