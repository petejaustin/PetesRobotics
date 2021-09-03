import rospy
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs

def t_pose(input, curr_frame, desired_frame):

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(2)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.header.frame_id = curr_frame
    pose_stamped.pose = input
    pose_stamped.header.stamp = rospy.Time.now()

    output_pose_stamped = tf_buffer.transform(pose_stamped, desired_frame, rospy.Duration(1))
    return output_pose_stamped.pose

# Test code
rospy.init_node("foo")

my_pose = Pose()
my_pose.position.x = 0.25
my_pose.position.y = 0.50
my_pose.position.z = 1.50
my_pose.orientation.x = 0.634277921154
my_pose.orientation.y = 0.597354098852
my_pose.orientation.z = 0.333048372508
my_pose.orientation.w = 0.360469667089

transformed_pose = t_pose(my_pose, "panda_link7", "world")

print("POSE VALUES AT LINK7 REFERENCE FRAME:\n", my_pose)
print("\nPOSE VALUES AT WORLD REFERENCE FRAME:\n", transformed_pose)