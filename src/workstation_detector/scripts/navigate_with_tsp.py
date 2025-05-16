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
from tsp import TSP

def get_line_midpoints(lines):
    """
    Extract midpoints from detected lines for TSP calculation
    Args:
        lines: List of detected lines from WorkstationDetector
    Returns:
        List of (x, y) midpoint coordinates
    """
    midpoints = []
    for line in lines:
        start = np.array(line['start'])
        end = np.array(line['end'])
        midpoint = (start + end) / 2
        midpoints.append((float(midpoint[0]), float(midpoint[1])))
    
    return midpoints

def main():
    rospy.init_node('navigate_with_tsp')
    rate = rospy.Rate(5)  # 5 Hz check rate
    
    rospy.loginfo("Starting WorkstationDetector...")
    detector = WorkstationDetector()
    
    # Wait for lines to be detected
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
    
    # Get lines information
    lines = detector.latest_lines
    rospy.loginfo(f"Found {len(lines)} lines to visit")
    
    # Get midpoints of all lines for TSP calculation
    midpoints = get_line_midpoints(lines)
    
    # Add robot's current position (0,0) as the starting point for TSP
    current_pos = (0.0, 0.0)  # Robot is at origin in its own frame
    cities = [current_pos] + midpoints
    
    # Initialize TSP solver
    rospy.loginfo("Running TSP solver to find optimal path...")
    tsp_solver = TSP(cities)
    
    # Find the best route
    tsp_solver.find_best_route()
    
    # Get optimal route
    route, distance = tsp_solver.get_best_route()
    
    # Skip the first point in the route as it's the robot's current position
    route = route[1:]
    
    rospy.loginfo(f"TSP solver found optimal route with total distance: {distance:.2f}m")
    
    # Create mapping from midpoints back to line indices
    midpoint_to_index = {}
    for i, midpoint in enumerate(midpoints):
        midpoint_to_index[midpoint] = i
    
    # Visit each line in the optimal order
    rospy.loginfo("Starting navigation to lines in optimal order...")
    
    for i, point in enumerate(route):
        # Find which line this midpoint belongs to
        line_index = midpoint_to_index[point]
        
        rospy.loginfo(f"Navigating to line {line_index} ({i+1}/{len(route)})")
        
        # Navigate to the line using direct velocity commands
        success = navigate_to_line_with_cmd_vel(detector, line_index)
        
        if success:
            rospy.loginfo(f"Successfully navigated to line {line_index}")
        else:
            rospy.logerr(f"Failed to navigate to line {line_index}")
        
        # Small pause between navigation commands
        rospy.sleep(2.0)
    
    rospy.loginfo("Completed visiting all lines in optimal order!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
