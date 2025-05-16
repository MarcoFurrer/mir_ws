#!/usr/bin/env python3

import rospy
import sys
import os
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import quaternion_from_euler

# Add the directory of this script to the Python path to import WorkstationDetector
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

# Import the WorkstationDetector class
from workstation_detector import WorkstationDetector
from navigate_to_line_with_cmd_vel import navigate_to_line_with_cmd_vel

def main():
    # Initialize ROS node
    rospy.init_node('navigate_to_first_line')
    
    rospy.loginfo("Starting WorkstationDetector...")
    
    # Create an instance of WorkstationDetector
    detector = WorkstationDetector()
    
    # Wait for lines to be detected
    rate = rospy.Rate(5)  # 1 Hz check rate
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
    
    # Navigate using direct velocity commands
    success = navigate_to_line_with_cmd_vel(detector, 0)
    
    if success:
        rospy.loginfo("Navigation command completed successfully!")
    else:
        rospy.logerr("Failed to navigate to line.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass