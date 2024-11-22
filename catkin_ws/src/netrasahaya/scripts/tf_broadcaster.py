#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from geometry_msgs.msg import Twist, WrenchStamped
import message_filters
import math

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

def handle_t265_odom(msg, now = None):
    base_link = TransformStamped()
    base_link.header.stamp = rospy.Time.now() if now == None else now
    base_link.header.frame_id = "odom"
    base_link.child_frame_id = "base_link"

    position = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    rot_matrix = tf_conversions.transformations.euler_matrix(0, 0, math.pi/2, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_position = quat_rotate(q_rot, position)

    base_link.transform.translation.x = new_position[0]
    base_link.transform.translation.y = new_position[1]
    base_link.transform.translation.z = new_position[2]
    
    new_orientation = tf_conversions.transformations.quaternion_multiply(orientation, q_rot)

    base_link.transform.rotation.x = new_orientation[0]
    base_link.transform.rotation.y = new_orientation[1]
    base_link.transform.rotation.z = new_orientation[2]
    base_link.transform.rotation.w = new_orientation[3]

    # base_link.transform.translation.x = rot_matrix[0][0]*position[0] + rot_matrix[0][1]*position[1] + rot_matrix[0][2]*position[2]
    # base_link.transform.translation.y = rot_matrix[1][0]*position[0] + rot_matrix[1][1]*position[1] + rot_matrix[1][2]*position[2]
    # base_link.transform.translation.z = rot_matrix[2][0]*position[0] + rot_matrix[2][1]*position[1] + rot_matrix[2][2]*position[2]
    
    # new_orientation = tf_conversions.transformations.quaternion_multiply(orientation, tf_conversions.transformations.quaternion_from_matrix(rot_matrix))

    # base_link.transform.rotation.x = new_orientation[0]
    # base_link.transform.rotation.y = new_orientation[1]
    # base_link.transform.rotation.z = new_orientation[2]
    # base_link.transform.rotation.w = new_orientation[3]

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

    new_translation = quat_rotate(q_rot, translation)

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

    new_origin_position = quat_rotate(q_rot, origin_position)
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

def handle_d400_image(msg, now = None):
    camera_image = Image()

    camera_image.header.stamp = rospy.Time.now() if now == None else now
    camera_image.header.frame_id = "camera_link"
    camera_image.height = msg.height
    camera_image.width = msg.width
    camera_image.encoding = msg.encoding
    camera_image.is_bigendian = msg.is_bigendian
    camera_image.step = msg.step
    camera_image.data = msg.data

    image_pub.publish(camera_image)

def handle_d400_camera_info(msg, now = None):
    camera_info = CameraInfo()

    camera_info.header.stamp = rospy.Time.now() if now == None else now
    camera_info.header.frame_id = "camera_link"
    camera_info.height = msg.height
    camera_info.width = msg.width
    camera_info.distortion_model = msg.distortion_model
    camera_info.D = msg.D
    camera_info.K = msg.K
    camera_info.R = msg.R
    camera_info.P = msg.P
    camera_info.binning_x = msg.binning_x
    camera_info.binning_y = msg.binning_y
    camera_info.roi = msg.roi

    camera_info_pub.publish(camera_info)

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

def callback(odom, points, image, camera_info):
    now = rospy.Time.now()
    # handle_t265_odom(odom, now)
    # handle_d400_points(points, now)
    handle_d400_image(image, now)
    handle_d400_camera_info(camera_info, now)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    br = tf2_ros.TransformBroadcaster()

    rospy.Subscriber('/t265/odom/sample', Odometry, handle_t265_odom)
    rospy.Subscriber('/d400/depth/color/points', PointCloud2, handle_d400_points)
    # rospy.Subscriber('/d400/depth/image_rect_raw', Image, handle_d400_image)
    # rospy.Subscriber('/d400/depth/camera_info', CameraInfo, handle_d400_camera_info)
    rospy.Subscriber('/occupancy', OccupancyGrid, handle_occupancy)
    rospy.Subscriber('/cmd_vel', Twist, handle_twist)

    # odometry_sub = message_filters.Subscriber('/t265/odom/sample', Odometry)
    # point_cloud_sub = message_filters.Subscriber('/d400/depth/color/points', PointCloud2)
    # image_sub = message_filters.Subscriber('/d400/depth/image_rect_raw', Image)
    # camera_info_sub = message_filters.Subscriber('/d400/depth/camera_info', CameraInfo)

    # ts = message_filters.ApproximateTimeSynchronizer([odometry_sub, point_cloud_sub, image_sub, camera_info_sub], 10, 0.1)
    # ts.registerCallback(callback)

    odometry_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    point_cloud_pub = rospy.Publisher('/depth', PointCloud2, queue_size=10)
    # image_pub = rospy.Publisher('/depth/image', Image, queue_size=10)
    # camera_info_pub = rospy.Publisher('/depth/camera_info', CameraInfo, queue_size=10)
    map_pub = rospy.Publisher('/occupancy/map', OccupancyGrid, queue_size=10)
    twist_pub = rospy.Publisher('/cmd_vel/stamped', WrenchStamped, queue_size=10)

    rospy.spin()
