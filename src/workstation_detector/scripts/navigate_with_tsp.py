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
    
    # Test basic movement to ensure velocity commands work
    rospy.loginfo("Testing basic robot movement...")
    test_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(1.0)  # Wait for publisher connections
    
    # Simple forward test
    forward_cmd = Twist()
    forward_cmd.linear.x = 0.1  # Slow forward
    
    rospy.loginfo("Testing forward movement for 1 second...")
    start_time = time.time()
    while time.time() - start_time < 1.0 and not rospy.is_shutdown():
        test_pub.publish(forward_cmd)
        rospy.sleep(0.1)
    
    # Stop
    test_pub.publish(Twist())
    rospy.sleep(1.0)
    
    # Simple rotation test
    turn_cmd = Twist()
    turn_cmd.angular.z = 0.3  # Slow rotation
    
    rospy.loginfo("Testing rotation for 1 second...")
    start_time = time.time()
    while time.time() - start_time < 1.0 and not rospy.is_shutdown():
        test_pub.publish(turn_cmd)
        rospy.sleep(0.1)
    
    # Stop again
    test_pub.publish(Twist())
    rospy.sleep(1.0)
    
    # Continue with regular execution
    rospy.loginfo("Basic movement test complete. Starting line detection...")
    
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
    
    # Simplified approach: directly use the ordered lines from TSP solution
    visit_order = []
    for point in route:
        # Find which line this TSP point corresponds to
        closest_idx = -1
        closest_dist = float('inf')
        
        # Find the closest line midpoint to this TSP coordinate
        for i, line in enumerate(lines):
            mid_x = (line['start'][0] + line['end'][0]) / 2
            mid_y = (line['start'][1] + line['end'][1]) / 2
            
            dist = math.sqrt((point[0] - mid_x)**2 + (point[1] - mid_y)**2)
            
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        
        if closest_idx >= 0 and closest_dist < 1.0:  # Only add if it's reasonably close
            visit_order.append(closest_idx)
    
    # Remove duplicates but preserve order
    ordered_indices = []
    for idx in visit_order:
        if idx not in ordered_indices:
            ordered_indices.append(idx)
    
    # Make sure we have a clean publisher for velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(1.0)  # Wait for publisher to connect
    
    # Log the navigation plan
    rospy.loginfo(f"Starting navigation to {len(ordered_indices)} lines in optimal order...")
    for i, line_idx in enumerate(ordered_indices):
        line = lines[line_idx]
        rospy.loginfo(f"  Line {i+1}: length={line['length']:.2f}m, " +
                     f"midpoint=({(line['start'][0]+line['end'][0])/2:.2f}, " +
                     f"{(line['start'][1]+line['end'][1])/2:.2f})")
    
    # Navigate to each line in order
    for i, line_idx in enumerate(ordered_indices):
        rospy.loginfo(f"Navigating to line {i+1}/{len(ordered_indices)}")
        
        # Simply use the index directly - much simpler and more reliable
        target_idx = line_idx
        
        # Now we navigate to the target line directly - no complex matching needed
        rospy.loginfo(f"Navigating to line {target_idx} ({i+1}/{len(ordered_indices)})")
        
        # Navigate to the line using direct velocity commands with more debug output
        try:
            rospy.loginfo(f"Attempting to navigate to line {target_idx}...")
            success = navigate_to_line_with_cmd_vel(detector, target_idx)
            
            if success:
                rospy.loginfo(f"Successfully navigated to line {target_idx}")
            else:
                rospy.logerr(f"Failed to navigate to line {target_idx}")
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
    
