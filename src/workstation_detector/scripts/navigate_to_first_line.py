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
    
    # Navigate using direct velocity commands
    success = navigate_to_line_with_cmd_vel(detector, 0)
    
    if success:
        rospy.loginfo("Navigation command completed successfully!")
    else:
        rospy.logerr("Failed to navigate to line.")

def navigate_to_line_with_cmd_vel(detector, line_index=0, distance_from_line=0.7):
    """
    Navigate to a line using direct velocity commands
    
    Args:
        detector: WorkstationDetector instance
        line_index: Index of the line to navigate to
        distance_from_line: How far from the line to position the robot (meters)
        
    Returns:
        True if navigation command was executed successfully
    """
    # Check if line exists
    if line_index >= len(detector.latest_lines):
        rospy.logwarn(f"Line index {line_index} is out of range")
        return False
        
    # Get the line
    line = detector.latest_lines[line_index]
    
    # Create velocity publisher
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Ensure publisher is connected
    rospy.sleep(1.0)
    
    # Get line information for logging
    line_angle = line['angle']
    line_length = line['length']
    rospy.loginfo(f"Moving toward line with length={line_length:.2f}m, angle={line_angle:.1f}°")
    
    # Calculate angle to midpoint of line
    start = np.array(line['start'])
    end = np.array(line['end'])
    midpoint = (start + end) / 2
    angle_to_midpoint = math.atan2(midpoint[1], midpoint[0])
    distance_to_midpoint = math.sqrt(midpoint[0]**2 + midpoint[1]**2)
    
    rospy.loginfo(f"Line midpoint at [{midpoint[0]:.2f}, {midpoint[1]:.2f}], " +
                 f"distance={distance_to_midpoint:.2f}m, angle={math.degrees(angle_to_midpoint):.1f}°")
    
    # Calculate drive distance (stop before reaching the line)
    drive_distance = max(0.1, distance_to_midpoint - distance_from_line)
    
    # Phase 1: Rotation - turn toward the line
    rospy.loginfo("Phase 1: Turning to face the line...")
    
    turn_cmd = Twist()
    turn_cmd.angular.z = 0.2 if angle_to_midpoint > 0 else -0.2  # Turn at 0.2 rad/s
    
    # Turn for approximately the right amount of time
    turn_time = abs(angle_to_midpoint) / abs(turn_cmd.angular.z)
    turn_time = min(turn_time, 5.0)  # Cap turn time at 5 seconds for safety
    
    rospy.loginfo(f"Turning for {turn_time:.1f} seconds to face angle {math.degrees(angle_to_midpoint):.1f}°")
    
    # Publish turn command for the calculated duration
    start_time = time.time()
    rate = rospy.Rate(10)  # 10 Hz control loop
    while time.time() - start_time < turn_time and not rospy.is_shutdown():
        cmd_vel_pub.publish(turn_cmd)
        rate.sleep()
    
    # Stop turning
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)
    rospy.sleep(1.0)  # Pause briefly to stabilize
    
    # Phase 2: Move forward toward the line
    rospy.loginfo(f"Phase 2: Moving forward {drive_distance:.2f} meters...")
    
    # Set forward speed
    forward_speed = 0.1  # m/s
    drive_time = drive_distance / forward_speed
    drive_time = min(drive_time, 10.0)  # Cap at 10 seconds for safety
    
    forward_cmd = Twist()
    forward_cmd.linear.x = forward_speed
    
    # Publish forward command for the calculated duration
    start_time = time.time()
    while time.time() - start_time < drive_time and not rospy.is_shutdown():
        cmd_vel_pub.publish(forward_cmd)
        rate.sleep()
    
    # Stop the robot
    cmd_vel_pub.publish(stop_cmd)
    rospy.loginfo("Movement completed")
    
    return True

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass