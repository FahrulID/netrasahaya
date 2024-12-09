import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf_conversions
import math

import utils.utils as utils

def go_to_goal(goal_x, goal_y):
    # Start a connection to the move_base action server
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Set the goal for A*
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.w = 1.0  # Facing forward

    rospy.loginfo("Sending goal to move_base...")
    client.send_goal(goal)

    # DWA and A* will work together under the hood in ROS's move_base
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Reached the goal!")
    else:
        rospy.loginfo("Failed to reach the goal...")

if __name__ == '__main__':
    rospy.init_node('blind_navigation_example')

    # Let's say the goal is at (5, 5) in the map
    # go_to_goal(5, 5)

    # print(utils.utils.get_relative_quaternion([0, 0, 0, 1], [0, 0, -0.707, 0.707]))  # Should print [0, 0, 1]

    origin_position = [-2.950000, 3.900000, 0]
    origin_orientation = [0, 0, -0.724,  0.690]

    rot_matrix = tf_conversions.transformations.euler_matrix(0, 0, math.pi/2, 'rxyz')
    q_rot = tf_conversions.transformations.quaternion_from_matrix(rot_matrix)

    new_origin_position = utils.quat_rotate(q_rot, origin_position)
    new_origin_orientation = tf_conversions.transformations.quaternion_multiply(origin_orientation, q_rot)

    print(new_origin_position)
    print(new_origin_orientation)
