#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion

def handle_t265_odom(msg):
    # Broadcast map -> odom (static)
    static_broadcaster.sendTransform(map_to_odom_tf)
    
    # Broadcast odom -> base_link (dynamic, from t265 odom)
    odom_to_base_link_tf = TransformStamped()
    odom_to_base_link_tf.header.stamp = rospy.Time.now()
    odom_to_base_link_tf.header.frame_id = "odom"
    odom_to_base_link_tf.child_frame_id = "base_link"
    
    # Set the position and orientation from the t265 odometry message
    odom_to_base_link_tf.transform.translation.x = msg.pose.pose.position.x
    odom_to_base_link_tf.transform.translation.y = msg.pose.pose.position.y
    odom_to_base_link_tf.transform.translation.z = msg.pose.pose.position.z
    
    # Ensure quaternion is properly assigned
    odom_to_base_link_tf.transform.rotation = msg.pose.pose.orientation

    # Broadcast the transform
    dynamic_broadcaster.sendTransform(odom_to_base_link_tf)

    # Broadcast base_link -> t265_link (static)
    base_link_to_t265_tf.header.stamp = rospy.Time.now()
    dynamic_broadcaster.sendTransform(base_link_to_t265_tf)
    
    # Broadcast base_link -> d400_link (static)
    base_link_to_d400_tf.header.stamp = rospy.Time.now()
    dynamic_broadcaster.sendTransform(base_link_to_d400_tf)

def handle_map(msg):
    # x 0 y 0 z 0.707107 w -0.707107
    new_msg = msg
    new_msg.info.origin.position = msg.info.origin.position
    new_msg.info.origin.orientation.x = 0
    new_msg.info.origin.orientation.y = 0
    new_msg.info.origin.orientation.z = 0.707107
    new_msg.info.origin.orientation.w = -0.707107

    costmap.publish(new_msg)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    # Initialize the transform broadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    dynamic_broadcaster = tf2_ros.TransformBroadcaster()

    # Define the static transform from map to odom
    map_to_odom_tf = TransformStamped()
    map_to_odom_tf.header.stamp = rospy.Time.now()
    map_to_odom_tf.header.frame_id = "map"
    map_to_odom_tf.child_frame_id = "odom"
    map_to_odom_tf.transform.translation.x = 0.0
    map_to_odom_tf.transform.translation.y = 0.0
    map_to_odom_tf.transform.translation.z = 0.0
    # Convert quaternion from numpy array to ROS Quaternion message
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    map_to_odom_tf.transform.rotation = Quaternion(*q)

    # Define the static transform from base_link to t265_link
    base_link_to_t265_tf = TransformStamped()
    base_link_to_t265_tf.header.frame_id = "base_link"
    base_link_to_t265_tf.child_frame_id = "tracking_link"
    base_link_to_t265_tf.transform.translation.x = 0.0
    base_link_to_t265_tf.transform.translation.y = 0.0
    base_link_to_t265_tf.transform.translation.z = 0.0
    base_link_to_t265_tf.transform.rotation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    # Define the static transform from base_link to d400_link
    base_link_to_d400_tf = TransformStamped()
    base_link_to_d400_tf.header.frame_id = "base_link"
    base_link_to_d400_tf.child_frame_id = "depth_link"
    base_link_to_d400_tf.transform.translation.x = 0.0
    base_link_to_d400_tf.transform.translation.y = 0.0
    base_link_to_d400_tf.transform.translation.z = 0.0
    base_link_to_d400_tf.transform.rotation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    # Subscribe to the /t265/odom/sample topic (nav_msgs/Odometry)
    rospy.Subscriber('/t265/odom/sample', Odometry, handle_t265_odom)
    # rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, handle_map)
    # costmap = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=10)

    rospy.spin()
