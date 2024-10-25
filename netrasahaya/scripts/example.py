import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

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
    go_to_goal(5, 5)
