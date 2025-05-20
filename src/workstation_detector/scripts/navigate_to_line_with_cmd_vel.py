#!/usr/bin/env python3

import rospy
import sys
import os
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Add the directory of this script to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

def navigate_to_pose_with_cmd_vel(pose, frame_id='base_link'):
    """
    Navigate to a pose using direct velocity commands
    
    Args:
        pose: geometry_msgs/Pose target pose
        frame_id: Frame ID of the pose
        
    Returns:
        True if navigation command was executed successfully
    """
    rospy.loginfo("Starting navigation to pose...")
        
    # Create velocity publisher
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Ensure publisher is connected
    rospy.sleep(1.0)
    
    # Create velocity publisher
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    # Ensure publisher is connected
    rospy.sleep(1.0)
    
    # Get pose information for logging
    position = pose.position
    orientation = pose.orientation
    
    # Extract yaw angle from quaternion
    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    
    rospy.loginfo(f"Moving toward pose at ({position.x:.2f}, {position.y:.2f}) with orientation {math.degrees(yaw):.1f}°")
    
    # Calculate angle and distance to target position
    angle_to_target = math.atan2(position.y, position.x)
    distance_to_target = math.sqrt(position.x**2 + position.y**2)
    
    rospy.loginfo(f"Target at distance={distance_to_target:.2f}m, angle={math.degrees(angle_to_target):.1f}°")
    
    # Phase 1: Rotation - turn toward the target
    rospy.loginfo("Phase 1: Turning to face the target...")
    
    turn_cmd = Twist()
    turn_cmd.angular.z = 0.2 if angle_to_target > 0 else -0.2  # Turn at 0.2 rad/s
    
    # Turn for approximately the right amount of time
    turn_time = abs(angle_to_target) / abs(turn_cmd.angular.z)
    turn_time = min(turn_time, 5.0)  # Cap turn time at 5 seconds for safety
    
    rospy.loginfo(f"Turning for {turn_time:.1f} seconds to face angle {math.degrees(angle_to_target):.1f}°")
    
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
    
    # Phase 2: Move forward toward the target
    drive_distance = distance_to_target
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
    
    # Phase 3: Rotate to the final orientation
    final_rotation = yaw - angle_to_target
    rospy.loginfo(f"Phase 3: Rotating to final orientation {math.degrees(yaw):.1f}°...")
    
    # Set rotation speed and direction
    turn_cmd = Twist()
    turn_cmd.angular.z = 0.2 if final_rotation > 0 else -0.2
    
    # Turn for approximately the right amount of time
    turn_time = abs(final_rotation) / abs(turn_cmd.angular.z)
    turn_time = min(turn_time, 5.0)  # Cap turn time at 5 seconds for safety
    
    # Publish turn command for the calculated duration
    start_time = time.time()
    while time.time() - start_time < turn_time and not rospy.is_shutdown():
        cmd_vel_pub.publish(turn_cmd)
        rate.sleep()
    
    # Stop the robot
    cmd_vel_pub.publish(stop_cmd)
    rospy.loginfo("Navigation to pose completed")
    
    return True

def navigate_to_line_with_cmd_vel(detector, line_index=0, distance_from_line=0.4):
    """
    Legacy method to maintain compatibility with existing code
    
    Args:
        detector: WorkstationDetector instance
        line_index: Index of the line to navigate to
        distance_from_line: How far from the line to position the robot (meters)
        
    Returns:
        True if navigation command was executed successfully
    """
    rospy.loginfo("Starting legacy navigation to line...")
    
    # Check if line exists
    if not hasattr(detector, 'latest_lines') or line_index >= len(detector.latest_lines):
        rospy.logwarn(f"Line index {line_index} is out of range")
        return False
    
    # Get the line
    line = detector.latest_lines[line_index]
    
    # Calculate midpoint of the line
    line_start = np.array(line['start'])
    line_end = np.array(line['end'])
    line_midpoint = (line_start + line_end) / 2.0
    
    # Calculate line direction vector
    line_direction = line_end - line_start
    line_length = np.linalg.norm(line_direction)
    
    if line_length < 0.01:  # Avoid division by zero
        rospy.logwarn("Line is too short for navigation")
        return False
    
    # Normalize line direction
    line_direction = line_direction / line_length
    
    # Calculate normal vector to the line (perpendicular)
    # Rotate 90 degrees counter-clockwise
    line_normal = np.array([-line_direction[1], line_direction[0]])
    
    # Position in front of the line
    target_position = line_midpoint + line_normal * distance_from_line
    
    # Robot should face the line
    target_orientation = math.atan2(-line_normal[1], -line_normal[0])
    
    # Create a pose
    target_pose = Pose()
    target_pose.position.x = float(target_position[0])
    target_pose.position.y = float(target_position[1])
    target_pose.position.z = 0.0
    
    # Convert euler angle to quaternion
    quaternion = quaternion_from_euler(0, 0, target_orientation)
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]
    
    # Navigate to the calculated pose
    return navigate_to_pose_with_cmd_vel(target_pose)