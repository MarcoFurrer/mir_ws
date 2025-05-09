#!/usr/bin/env python3

import rospy
import sys
import os
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# Add the directory of this script to the Python path to import WorkstationDetector
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

# Import the WorkstationDetector class
from workstation_detector import WorkstationDetector

def main():
    # Initialize ROS node
    rospy.init_node('navigate_to_first_line')
    
    rospy.loginfo("Starting WorkstationDetector...")
    
    # Create an instance of WorkstationDetector
    detector = WorkstationDetector()
    
    # Wait for lines to be detected
    rate = rospy.Rate(1)  # 1 Hz check rate
    max_wait_time = 10  # Maximum wait time in seconds
    wait_count = 0
    
    rospy.loginfo("Waiting for lines to be detected...")
    
    while not rospy.is_shutdown() and wait_count < max_wait_time:
        if hasattr(detector, 'latest_lines') and detector.latest_lines:
            rospy.loginfo(f"Detected {len(detector.latest_lines)} lines")
            break
            
        rate.sleep()
        wait_count += 1
        rospy.loginfo(f"Waiting... {wait_count}/{max_wait_time} seconds")
    
    # Check if lines were detected
    if not hasattr(detector, 'latest_lines') or not detector.latest_lines:
        rospy.logerr("No lines detected within the timeout period. Exiting.")
        return
    
    # Navigate to the first line
    rospy.loginfo("Navigating to the first detected line...")
    
    # Get the first line
    first_line = detector.latest_lines[0]
    rospy.loginfo(f"First line: length={first_line['length']:.2f}m, angle={first_line['angle']:.1f}°")
    
    # Navigate using transformed goal
    success = navigate_to_line_base_frame(detector, 0)
    
    if success:
        rospy.loginfo("Navigation command sent successfully!")
    else:
        rospy.logerr("Failed to send navigation command.")

def navigate_to_line_base_frame(detector, line_index=0, distance_from_line=0.7):
    """
    Navigate to a line using the base_link frame for better navigation
    
    Args:
        detector: WorkstationDetector instance
        line_index: Index of the line to navigate to
        distance_from_line: How far from the line to position the robot (meters)
        
    Returns:
        True if navigation command was sent successfully
    """
    # Check if line exists
    if line_index >= len(detector.latest_lines):
        rospy.logwarn(f"Line index {line_index} is out of range")
        return False
        
    # Get the line
    line = detector.latest_lines[line_index]
    
    # Calculate line midpoint and direction
    start = np.array(line['start'])
    end = np.array(line['end'])
    midpoint = (start + end) / 2
    
    # We need to transform this goal to be relative to the robot's base frame
    # Since the laser scan is already in the laser frame, we need to consider
    # where the laser is mounted on the robot
    
    # For MIR robots, the laser is typically at the front or back
    # Let's create a navigation goal in base_link frame
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    # Ensure publisher is connected
    rospy.sleep(0.5)
    
    # Create the goal
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    
    # Use base_link as the reference frame for more reliable navigation
    goal.header.frame_id = "base_link"
    
    # Calculate distance and direction to the line
    distance_to_midpoint = math.sqrt(midpoint[0]**2 + midpoint[1]**2)
    
    # Calculate the angle to the midpoint for better orientation
    angle_to_midpoint = math.atan2(midpoint[1], midpoint[0])
    
    # Move in the direction of the midpoint, but stop before reaching it
    # Using the base_link frame (robot-centric)
    target_distance = max(0.5, distance_to_midpoint - distance_from_line)
    
    goal.pose.position.x = target_distance * math.cos(angle_to_midpoint)
    goal.pose.position.y = target_distance * math.sin(angle_to_midpoint)
    goal.pose.position.z = 0.0
    
    # Orient the robot to face the line
    # The line's angle in the laser frame
    line_angle = math.radians(line['angle'])
    
    # Calculate desired orientation: the robot should face perpendicular to the line
    # Adding PI/2 to make the robot face the line perpendicularly
    desired_orientation = line_angle + (math.pi/2)
    
    # Convert to quaternion
    q = quaternion_from_euler(0, 0, desired_orientation)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]
    
    # Publish the goal
    goal_pub.publish(goal)
    rospy.loginfo(f"Published navigation goal: distance={target_distance:.2f}m, angle={math.degrees(angle_to_midpoint):.1f}°")
    
    return True

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass