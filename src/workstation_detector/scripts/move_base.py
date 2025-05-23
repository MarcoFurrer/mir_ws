#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

if __name__ == "__main__":
    rospy.init_node("move_base_forward")
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.sleep(1.0)  # Wait for publisher connection

    # Create a goal 1 meter forward in the robot's current frame
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_link"  # Send goal relative to robot's current pose
    goal.pose.position.x = 1.0
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, 0)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    rospy.loginfo("Publishing goal 1 meter forward...")
    goal_pub.publish(goal)
    rospy.loginfo("Goal published. The robot should start moving forward.")
    rospy.sleep(2.0)  # Give time for the message to be sent
