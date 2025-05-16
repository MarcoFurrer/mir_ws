#!/usr/bin/env python3

import rospy
import sys
import os
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray

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
    
def filter_lines(lines, min_length=0.4, max_length=2.0):
    """
    Filter out lines that are likely walls based on length
    
    Args:
        lines: List of detected lines
        min_length: Minimum line length to consider
        max_length: Maximum line length to consider (walls are often longer)
        
    Returns:
        List of lines that are likely workstations (not walls)
    """
    filtered_lines = []
    for line in lines:
        # Filter based on length
        if min_length <= line['length'] <= max_length:
            # Additional filtering based on angle - avoid lines too close to cardinal directions
            angle = line['angle'] % 180
            angle_tolerance = 5
            # Skip lines that are aligned with walls (approximately 0°, 90°, or 180°)
            if (angle < angle_tolerance or
                abs(angle - 90) < angle_tolerance or
                abs(angle - 180) < angle_tolerance):
                rospy.loginfo(f"Skipping potential wall with angle {angle:.1f}° and length {line['length']:.2f}m")
                continue
                
            filtered_lines.append(line)
            
    rospy.loginfo(f"Filtered {len(lines)} lines to {len(filtered_lines)} potential workstations")
    return filtered_lines

def main():
    rospy.init_node('navigate_with_tsp')
    rate = rospy.Rate(5)  # 5 Hz check rate
    
    # Get parameters from ROS parameter server with defaults
    min_line_length = rospy.get_param('~min_line_length', 0.4)
    max_line_length = rospy.get_param('~max_line_length', 1.5)
    max_wait_time = rospy.get_param('~wait_time', 15)
    min_wait_time = rospy.get_param('~min_wait_time', 5)
    
    rospy.loginfo(f"Starting TSP navigation with parameters:")
    rospy.loginfo(f"  - Minimum line length: {min_line_length}m")
    rospy.loginfo(f"  - Maximum line length: {max_line_length}m") 
    rospy.loginfo(f"  - Maximum wait time: {max_wait_time}s")
    rospy.loginfo(f"  - Minimum wait time: {min_wait_time}s")
    
    rospy.loginfo("Starting WorkstationDetector...")
    detector = WorkstationDetector()
    
    # Wait for lines to be detected
    wait_count = 0
    
    rospy.loginfo("Waiting for lines to be detected...")
    
    while not rospy.is_shutdown() and wait_count < max_wait_time:
        if hasattr(detector, 'latest_lines') and detector.latest_lines:
            rospy.loginfo(f"Detected {len(detector.latest_lines)} lines")
            if wait_count < min_wait_time:  # Always wait at least minimum time to get more scans
                rospy.loginfo("Waiting for more scans...")
            else:
                break
            
        rate.sleep()
        wait_count += 1
        rospy.loginfo(f"Waiting... {wait_count}/{max_wait_time} seconds")
    
    # Check if lines were detected
    if not hasattr(detector, 'latest_lines') or not detector.latest_lines:
        rospy.logerr("No lines detected within the timeout period. Exiting.")
        return
    
    # Get lines information and filter out walls
    all_lines = detector.latest_lines
    lines = filter_lines(all_lines, min_length=min_line_length, max_length=max_line_length)
    
    # Publish visualization markers for walls vs. workstations
    marker_pub = rospy.Publisher('filtered_workstations', MarkerArray, queue_size=10)
    
    # Create visualization of filtered workstations (green) vs walls (red)
    try:
        marker_array = MarkerArray()
        
        # Delete previous markers
        delete_marker = Marker()
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.header.frame_id = detector.latest_frame_id
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Add workstation markers (green)
        for i, line in enumerate(lines):
            marker = Marker()
            marker.header.frame_id = detector.latest_frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "workstations"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.06  # Line width
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            start_point = Point()
            start_point.x = line['start'][0]
            start_point.y = line['start'][1]
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = line['end'][0]
            end_point.y = line['end'][1]
            end_point.z = 0.0
            
            marker.points.append(start_point)
            marker.points.append(end_point)
            marker_array.markers.append(marker)
        
        # Add wall markers (red)
        for i, line in enumerate([l for l in all_lines if l not in lines]):
            marker = Marker()
            marker.header.frame_id = detector.latest_frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "walls"
            marker.id = i + len(lines)  # Start IDs after workstations
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.04  # Line width
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.6  # Semi-transparent
            
            start_point = Point()
            start_point.x = line['start'][0]
            start_point.y = line['start'][1]
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = line['end'][0]
            end_point.y = line['end'][1]
            end_point.z = 0.0
            
            marker.points.append(start_point)
            marker.points.append(end_point)
            marker_array.markers.append(marker)
        
        # Publish markers
        marker_pub.publish(marker_array)
        rospy.loginfo(f"Published visualization of {len(lines)} workstations and {len(all_lines) - len(lines)} walls")
    except Exception as e:
        rospy.logwarn(f"Error publishing visualization markers: {str(e)}")
    
    if not lines:
        rospy.logwarn("No suitable workstation lines found after filtering. Using all detected lines.")
        lines = all_lines
        
    rospy.loginfo(f"Found {len(lines)} workstation lines to visit")
    
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
    
    # Create mapping between TSP cities and lines
    line_indices = []
    for point in route:
        # Find the closest line midpoint to this TSP point
        best_idx = -1
        best_dist = float('inf')
        
        for i, line in enumerate(lines):
            mid_x = (line['start'][0] + line['end'][0]) / 2
            mid_y = (line['start'][1] + line['end'][1]) / 2
            
            dist = math.sqrt((point[0] - mid_x)**2 + (point[1] - mid_y)**2)
            
            if dist < best_dist:
                best_dist = dist
                best_idx = i
                
        if best_idx >= 0:
            line_indices.append(best_idx)
    
    # Remove duplicates while preserving order
    unique_indices = []
    [unique_indices.append(idx) for idx in line_indices if idx not in unique_indices]
    
    # Visit each line in the optimal order
    rospy.loginfo(f"Starting navigation to {len(unique_indices)} lines in optimal order...")
    
    # Make sure we have a clean publisher for velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(1.0)  # Wait for publisher to connect
    
    for i, line_idx in enumerate(unique_indices):
        # Get updated detector data
        detector_line_idx = -1
        
        # Find matching line in current detector data
        for j, current_line in enumerate(detector.latest_lines):
            if j >= len(lines):
                continue
                
            if abs(current_line['length'] - lines[line_idx]['length']) < 0.1:
                # Similar length lines
                detector_line_idx = j
                break
        
        # If we can't find a matching line, use the original index if still valid
        if detector_line_idx < 0 and line_idx < len(detector.latest_lines):
            detector_line_idx = line_idx
            
        # Skip if no valid line found
        if detector_line_idx < 0 or detector_line_idx >= len(detector.latest_lines):
            rospy.logwarn(f"Cannot find line {line_idx} in current detector data, skipping")
            continue
            
        rospy.loginfo(f"Navigating to line {detector_line_idx} ({i+1}/{len(unique_indices)})")
        
        # Navigate to the line using direct velocity commands with more debug output
        try:
            success = navigate_to_line_with_cmd_vel(detector, detector_line_idx)
            
            if success:
                rospy.loginfo(f"Successfully navigated to line {detector_line_idx}")
            else:
                rospy.logerr(f"Failed to navigate to line {detector_line_idx}")
        except Exception as e:
            rospy.logerr(f"Error during navigation: {str(e)}")
            # Send a stop command
            stop_cmd = Twist()
            cmd_vel_pub.publish(stop_cmd)
            
        # Send an explicit stop command
        stop_cmd = Twist()
        cmd_vel_pub.publish(stop_cmd)
        
        # Small pause between navigation commands
        rospy.sleep(1.0)
    
    # Finish with a celebratory spin
    rospy.loginfo("Completed visiting all lines in optimal order! Doing a final rotation...")
    
    # Make one final rotation to show we're done
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(0.5)
    
    # Create a slow rotation command
    spin_cmd = Twist()
    spin_cmd.angular.z = 0.5  # Rotate at moderate speed
    
    # Rotate for 2 seconds
    start_time = time.time()
    rate = rospy.Rate(10)
    while time.time() - start_time < 2.0 and not rospy.is_shutdown():
        cmd_vel_pub.publish(spin_cmd)
        rate.sleep()
        
    # Stop spinning
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)
    
    rospy.loginfo("TSP navigation completed successfully!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in navigate_with_tsp: {str(e)}")
        # Always make sure we stop the robot if there's an error
        try:
            cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
            cmd_vel_pub.publish(Twist())
        except:
            pass
    
