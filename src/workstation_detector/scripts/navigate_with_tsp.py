#!/usr/bin/env python3

import rospy
import sys
import os
import math
from geometry_msgs.msg import PoseStamped, Twist, Point, PoseArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

# Add the directory of this script to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

# Import the navigation controller class and TSP solver
from navigation_with_cmd import NavigationWithCmd
from tsp import TSP

# Global variables to store received goal poses
workstation_goal_poses = None
workstation_frame_id = None
first_poses_received = False  # Flag to track if we've already received the first set

def workstation_poses_callback(msg):
    """
    Callback for receiving workstation goal poses - only stores the first set received
    
    Args:
        msg: PoseArray message containing goal poses for workstations
    """
    global workstation_goal_poses, workstation_frame_id, first_poses_received
    
    # Only accept the first set of poses we receive
    if not first_poses_received and len(msg.poses) > 0:
        workstation_goal_poses = msg.poses
        workstation_frame_id = msg.header.frame_id
        first_poses_received = True
        rospy.loginfo(f"Received workstation goal poses in frame '{workstation_frame_id}'")
        rospy.loginfo(f"Received first set of {len(workstation_goal_poses)} goal poses - will ignore further updates")
    elif not first_poses_received:
        rospy.loginfo(f"Received empty pose array, waiting for poses...")
    else:
        # We've already received our poses, so we'll ignore this update
        rospy.logdebug(f"Ignoring update with {len(msg.poses)} poses - using first set only")

def get_pose_positions(poses):
    """
    Extract position coordinates from a list of poses for TSP calculation
    
    Args:
        poses: List of geometry_msgs/Pose
    Returns:
        List of (x, y) position coordinates
    """
    positions = []
    for pose in poses:
        positions.append((float(pose.position.x), float(pose.position.y)))
    
    return positions
    

def main():
    rospy.init_node('navigate_with_tsp')
    rate = rospy.Rate(5)  # 5 Hz check rate
    
    # Create our navigation controller
    nav_controller = NavigationWithCmd()
    
    # Get parameters from ROS parameter server with defaults
    max_wait_time = rospy.get_param('~wait_time', 15)
    min_wait_time = rospy.get_param('~min_wait_time', 5)
    
    rospy.loginfo(f"Starting TSP navigation with parameters:")
    rospy.loginfo(f"  - Maximum wait time: {max_wait_time}s")
    rospy.loginfo(f"  - Minimum wait time: {min_wait_time}s")
    
    # Subscribe to workstation goal poses from WorkstationDetector
    # We only care about the first valid set of poses we receive
    rospy.Subscriber('workstation_goal_poses', PoseArray, workstation_poses_callback)
    rospy.loginfo("Waiting for first set of workstation goal poses to be published...")
    
    # Test the navigation controller
    rospy.loginfo("Testing navigation controller...")
    
    # Test odometry data reading
    x, y, yaw = nav_controller.get_odom_data()
    if x is not None and y is not None and yaw is not None:
        rospy.loginfo(f"Odometry test successful. Current position: ({x:.2f}, {y:.2f}), orientation: {math.degrees(yaw):.1f}°")
    else:
        rospy.logwarn("Could not get odometry data. Navigation may be limited.")
    
    # Continue with regular execution
    rospy.loginfo("Navigation controller test complete. Waiting for workstation poses...")
    
    # Wait for goal poses to be received
    wait_count = 0
    
    while not rospy.is_shutdown() and wait_count < max_wait_time:
        if workstation_goal_poses is not None and len(workstation_goal_poses) > 0:
            rospy.loginfo(f"Received {len(workstation_goal_poses)} goal poses")
            if wait_count < min_wait_time:  # Always wait at least minimum time to get more scans
                rospy.loginfo("Waiting for more scans...")
            else:
                break
            
        rate.sleep()
        wait_count += 1
        rospy.loginfo(f"Waiting... {wait_count}/{max_wait_time} seconds")
    
    # Check if goal poses were received
    if workstation_goal_poses is None or len(workstation_goal_poses) == 0:
        rospy.logerr("No workstation goal poses received within the timeout period. Exiting.")
        return
    
    # Publish visualization markers for goal poses
    marker_pub = rospy.Publisher('tsp_goal_poses', MarkerArray, queue_size=10)
    
    # Create visualization of goal poses
    try:
        marker_array = MarkerArray()
        
        # Delete previous markers
        delete_marker = Marker()
        delete_marker.header.stamp = rospy.Time.now()
        delete_marker.header.frame_id = workstation_frame_id
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Add markers for each goal pose
        for i, pose in enumerate(workstation_goal_poses):
            # Add arrow marker for pose
            marker = Marker()
            marker.header.frame_id = workstation_frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = "goal_poses"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.3  # Arrow length
            marker.scale.y = 0.05  # Arrow width
            marker.scale.z = 0.05  # Arrow height
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.5
            marker.color.a = 1.0
            
            marker.pose = pose
            marker_array.markers.append(marker)
            
            # Add text marker with index number
            text_marker = Marker()
            text_marker.header.frame_id = workstation_frame_id
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "goal_numbers"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.2  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.pose = pose
            text_marker.pose.position.z += 0.2  # Position text above the arrow
            text_marker.text = str(i)
            marker_array.markers.append(text_marker)
            
        # Publish markers
        marker_pub.publish(marker_array)
        rospy.loginfo(f"Published visualization of {len(workstation_goal_poses)} goal poses")
    except Exception as e:
        rospy.logwarn(f"Error publishing visualization markers: {str(e)}")
        
    rospy.loginfo(f"Found {len(workstation_goal_poses)} workstation goal poses to visit")
    
    # Get positions of all goal poses for TSP calculation
    goal_positions = get_pose_positions(workstation_goal_poses)
    
    # Add robot's current position (0,0) as the starting point for TSP
    current_pos = (0.0, 0.0)  # Robot is at origin in its own frame
    cities = [current_pos] + goal_positions
    
    # Initialize TSP solver
    rospy.loginfo(f"Running TSP solver to find optimal path...{cities} cities")
    
    tsp_solver = TSP(cities)
    
    # Find the best route
    tsp_solver.find_best_route()
    rospy.loginfo("TSP solver completed")
    # Get optimal route
    route, distance = tsp_solver.get_best_route()
    
    # Skip the first point in the route as it's the robot's current position
    route = route[1:]
    
    rospy.loginfo(f"TSP solver found optimal route with total distance: {distance:.2f}m")
    
    # Simplified approach: directly use the ordered poses from TSP solution
    visit_order = []
    for point in route:
        # Find which pose this TSP point corresponds to
        closest_idx = -1
        closest_dist = float('inf')
        
        # Find the closest pose to this TSP coordinate
        for i, pose in enumerate(workstation_goal_poses):
            pose_x = pose.position.x
            pose_y = pose.position.y
            
            dist = math.sqrt((point[0] - pose_x)**2 + (point[1] - pose_y)**2)
            
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
    
    # Make sure we have a clean publisher for velocity commands and goals
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    rospy.sleep(1.0)  # Wait for publishers to connect
    
    # Log the navigation plan
    rospy.loginfo(f"Starting navigation to {len(ordered_indices)} workstations in optimal order...")
    for i, pose_idx in enumerate(ordered_indices):
        pose = workstation_goal_poses[pose_idx]
        rospy.loginfo(f"  Workstation {i+1}: position=({pose.position.x:.2f}, {pose.position.y:.2f})")
    
    # Navigate to each pose in order
    for i, pose_idx in enumerate(ordered_indices):
        rospy.loginfo(f"Navigating to workstation {i+1}/{len(ordered_indices)}")
        
        # Get the target pose
        target_pose = workstation_goal_poses[pose_idx]
        
        # Create a PoseStamped message for visualization
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = workstation_frame_id
        goal_msg.pose = target_pose
        
        # Publish the goal for visualization
        rospy.loginfo(f"Navigating to workstation {pose_idx} ({i+1}/{len(ordered_indices)})")
        goal_pub.publish(goal_msg)
        
        # Use the navigation controller to navigate to the pose
        try:
            rospy.loginfo(f"Using navigation controller to navigate to workstation {pose_idx}...")
            # Use the improved navigation method with correction
            success = nav_controller.navigate_to_pose(target_pose, workstation_frame_id)
            
            if success:
                rospy.loginfo(f"Successfully navigated to workstation {pose_idx}")
            else:
                rospy.logerr(f"Failed to navigate to workstation {pose_idx}")
        except Exception as e:
            rospy.logerr(f"Error during navigation: {str(e)}")
            # Just continue to the next goal
        
        # Send a stop command to ensure robot is stopped
        stop_cmd = Twist()
        cmd_vel_pub.publish(stop_cmd)
            
        # Send an explicit stop command
        stop_cmd = Twist()
        cmd_vel_pub.publish(stop_cmd)
        
        # Small pause between navigation commands
        rospy.sleep(1.0)
    
    # Finish with a celebratory spin
    rospy.loginfo("Completed visiting all lines in optimal order! Doing a final rotation...")
    
    # Use the navigation controller to perform a full 360-degree spin
    nav_controller.rotate_in_place(math.radians(360))
    
    rospy.loginfo("TSP navigation completed successfully!")


if __name__ == '__main__':
    # Define nav_controller at global scope so it can be accessed in exception handling
    nav_controller = None
    
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
        except Exception as stop_error:
            rospy.logerr(f"Error stopping robot: {str(stop_error)}")
    
