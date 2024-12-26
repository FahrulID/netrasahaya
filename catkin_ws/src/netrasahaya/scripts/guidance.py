import rospy
import utils.utils as utils
from geometry_msgs.msg import Twist, TwistStamped
import tf2_ros

vel = Twist()
def vel_callback(msg):
    global vel
    vel = msg


class GuidanceDisabled(Exception):
    pass

if __name__ == '__main__':
    rospy.init_node('guidance')
    
    rospy.Subscriber('/cmd_vel', Twist, vel_callback)
    point_stamped_pub = rospy.Publisher('/guidance_twist', TwistStamped, queue_size=1)

    rate = rospy.Rate(30.0)
    br = tf2_ros.TransformBroadcaster()
    point_stamped = TwistStamped()
    seq = 0
    
    while not rospy.is_shutdown():
        try:
            enabled = rospy.get_param("/mode", "none") == "guidance"

            if not enabled:
                raise GuidanceDisabled("Guidance is disabled")
                
            if vel.linear.x == 0 and vel.linear.y == 0 and vel.linear.z == 0:
                raise GuidanceDisabled("Guidance is idle")
                
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.header.seq = seq
            point_stamped.header.frame_id = 't265_pose_frame'
            point_stamped.twist = vel
            # br.sendTransform(point_stamped)
            
            point_stamped_pub.publish(point_stamped)
            seq += 1
        except rospy.ROSInterruptException:
            break
        except GuidanceDisabled as e:
            pass
        except Exception as e:
            rospy.logerr(e)
            break
        
        rate.sleep()

    exit(0)