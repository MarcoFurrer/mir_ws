#!/usr/bin/env python3

import rospy
import sys
import os
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry

# Add the directory of this script to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)

class NavigationWithCmd:
    """
    A controller for robot navigation using odometry feedback and cmd_vel control
    """
    
    def __init__(self):
        """Initialize the navigation controller"""
        # Initialize publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # Parameters
        self.linear_speed = 0.15       # m/s - slightly faster but still safe
        self.angular_speed = 0.25      # rad/s - slightly faster rotations
        self.distance_tolerance = 0.15  # meters - more forgiving distance tolerance
        self.angle_tolerance = 0.08    # radians (≈4.6°) - slightly more forgiving angle tolerance
        self.rate = rospy.Rate(5)
        
        # Store the initial odometry position when the controller is created
        # This will be our reference for all waypoints
        self.odom_initial_position = self.get_odom_data()
        rospy.loginfo(f"Initial odometry position: ({self.odom_initial_position[0]:.2f}, {self.odom_initial_position[1]:.2f}, {self.odom_initial_position[2]:.2f})")
        
    def get_odom_data(self):
        """
        Get the current position and orientation from odometry
        
        Returns:
            tuple: (x, y, yaw) position and orientation, or (None, None, None) if failed
        """
        odom_data = {'x': None, 'y': None, 'yaw': None}
        
        def callback(msg):
            odom_data['x'] = msg.pose.pose.position.x
            odom_data['y'] = msg.pose.pose.position.y
            
            # Extract orientation as yaw angle
            quat = [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
            _, _, yaw = euler_from_quaternion(quat)
            odom_data['yaw'] = yaw
        
        # Subscribe to odometry
        odom_sub = rospy.Subscriber('/odom', Odometry, callback)
        
        # Wait for the data with timeout
        timeout = rospy.Time.now() + rospy.Duration(2.0)
        while (odom_data['x'] is None or odom_data['y'] is None or odom_data['yaw'] is None) and rospy.Time.now() < timeout:
            self.rate.sleep()
        
        # If we timed out, try once more with a longer timeout
        if odom_data['x'] is None or odom_data['y'] is None or odom_data['yaw'] is None:
            rospy.logwarn("Failed to get odometry data, retrying with longer timeout...")
            timeout = rospy.Time.now() + rospy.Duration(5.0)
            while (odom_data['x'] is None or odom_data['y'] is None or odom_data['yaw'] is None) and rospy.Time.now() < timeout:
                self.rate.sleep()
        
        # Unregister to avoid callback overhead
        odom_sub.unregister()
        
        return odom_data['x'], odom_data['y'], odom_data['yaw']

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        # More robust normalization using atan2
        return math.atan2(math.sin(angle), math.cos(angle))
        
    def transform_to_robot_frame(self, target_x, target_y):
        """
        Transform coordinates from the robot's initial reference frame to the current robot frame
        
        Args:
            target_x: X coordinate in the robot's initial reference frame
            target_y: Y coordinate in the robot's initial reference frame
            
        Returns:
            (x, y) coordinates relative to the robot's current position
        """
        # Get current robot position in odometry frame
        curr_x, curr_y, curr_yaw = self.get_odom_data()
        if curr_x is None:
            rospy.logerr("Failed to get odometry data for coordinate transformation")
            return None, None
            
        # Calculate robot's current position relative to its initial position
        initial_x, initial_y, initial_yaw = self.odom_initial_position
        
        # Displacement of the robot from initial position in odometry frame
        dx_odom = curr_x - initial_x
        dy_odom = curr_y - initial_y
        
        # Transform target from initial robot frame to odometry frame
        # First, rotate by initial_yaw to convert from robot's initial frame to odometry frame
        target_x_odom = initial_x + (target_x * math.cos(initial_yaw) - target_y * math.sin(initial_yaw))
        target_y_odom = initial_y + (target_x * math.sin(initial_yaw) + target_y * math.cos(initial_yaw))
        
        # Now transform from odometry frame to current robot frame
        # 1. Calculate vector from current robot position to target in odometry frame
        dx_target = target_x_odom - curr_x
        dy_target = target_y_odom - curr_y
        
        # 2. Rotate by -curr_yaw to convert from odometry frame to current robot frame
        robot_frame_x = dx_target * math.cos(-curr_yaw) - dy_target * math.sin(-curr_yaw)
        robot_frame_y = dx_target * math.sin(-curr_yaw) + dy_target * math.cos(-curr_yaw)
            
        return robot_frame_x, robot_frame_y

    def navigate_to_pose(self, pose, frame_id='base_link', max_attempts=2):
        """
        Navigate to a pose using odometry delta tracking approach
        
        Args:
            pose: geometry_msgs/Pose target pose in robot's initial reference frame
            frame_id: Frame ID of the pose
            max_attempts: Maximum number of correction attempts
            
        Returns:
            True if navigation was successful
        """
        rospy.loginfo("Starting navigation to pose...")
        
        # Extract target information
        target_x = pose.position.x
        target_y = pose.position.y
        
        # Extract orientation as yaw angle
        quat = [pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w]
        _, _, target_yaw = euler_from_quaternion(quat)
        
        rospy.loginfo(f"Target (in initial frame): position=({target_x:.2f}, {target_y:.2f}), orientation={math.degrees(target_yaw):.1f}°")
        
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                rospy.loginfo(f"Starting correction attempt {attempt}/{max_attempts}")
        
            # Transform the target coordinates from initial robot frame to current robot frame
            robot_frame_x, robot_frame_y = self.transform_to_robot_frame(target_x, target_y)
            if robot_frame_x is None:
                rospy.logerr("Failed to transform coordinates. Navigation aborted.")
                return False
                
            rospy.loginfo(f"Target (in current robot frame): position=({robot_frame_x:.2f}, {robot_frame_y:.2f})")
            
            # 1. Rotate to face target
            angle_to_target = math.atan2(robot_frame_y, robot_frame_x)
            success = self._rotate_to_angle(angle_to_target)
            if not success:
                rospy.logerr("Failed to rotate to target. Navigation aborted.")
                return False
            
            # 2. Move straight to target position
            distance = math.sqrt(robot_frame_x**2 + robot_frame_y**2)
            if distance < self.distance_tolerance:
                rospy.loginfo(f"Already within distance tolerance ({distance:.2f}m < {self.distance_tolerance:.2f}m), skipping forward movement")
            else:
                self._move_straight(distance)
            
            # 3. Rotate to final orientation in the global frame
            # We need to adjust the target orientation based on the robot's current orientation
            curr_x, curr_y, curr_yaw = self.get_odom_data()
            if curr_x is None or curr_y is None or curr_yaw is None:
                rospy.logerr("Failed to get current position for final orientation. Skipping final rotation.")
            else:
                # Transform the target yaw from initial frame to global frame
                initial_x, initial_y, initial_yaw = self.odom_initial_position
                global_target_yaw = self.normalize_angle(target_yaw + initial_yaw)
                
                # Calculate the required rotation in the robot's current frame
                rotation_needed = self.normalize_angle(global_target_yaw - curr_yaw)
                success = self._rotate_to_angle(rotation_needed)
                if not success:
                    rospy.logerr("Failed to rotate to final orientation. Navigation aborted.")
                    return False
            
            # Check if we're close enough to the target
            robot_frame_x, robot_frame_y = self.transform_to_robot_frame(target_x, target_y)
            if robot_frame_x is None:
                rospy.logerr("Failed to check position after navigation")
                continue
                
            error_distance = math.sqrt(robot_frame_x**2 + robot_frame_y**2)
            
            rospy.loginfo(f"Position error after attempt {attempt}: {error_distance:.2f}m")
            
            # If error is acceptable or this was the last attempt, we're done
            if error_distance <= self.distance_tolerance or attempt >= max_attempts:
                break
    
        rospy.loginfo("Navigation complete")
        return True
        
    def _rotate_to_angle(self, target_angle):
        """
        Rotate the robot to face a specific angle in the robot's local frame
        
        Args:
            target_angle: Angle to rotate to in radians (relative to current orientation)
            
        Returns:
            True if rotation was successful
        """
        # Normalize target angle to be between -π and π
        target_angle = self.normalize_angle(target_angle)
        rospy.loginfo(f"Rotating by angle {math.degrees(target_angle):.1f}°")
        
        # Get initial orientation
        _, _, initial_yaw = self.get_odom_data()
        if initial_yaw is None:
            rospy.logwarn("Failed to get orientation for rotation")
            return False
            
        # Calculate target orientation in odometry frame
        # Since target_angle is relative to current orientation, add it to the current orientation
        target_yaw = self.normalize_angle(initial_yaw + target_angle)
        
        rospy.loginfo(f"Initial orientation: {math.degrees(initial_yaw):.1f}°, " +
                      f"Target orientation: {math.degrees(target_yaw):.1f}°")
        
        # Check if we're already within tolerance
        if abs(target_angle) < self.angle_tolerance:
            rospy.loginfo(f"Already within angle tolerance ({math.degrees(abs(target_angle)):.1f}° < {math.degrees(self.angle_tolerance):.1f}°), skipping rotation")
            return True
        
        # Create twist command
        cmd = Twist()
        
        # Start rotation
        start_time = time.time()
        last_log_time = start_time
        
        # Rotate until we reach the target angle
        while not rospy.is_shutdown():
            # Get current orientation
            _, _, current_yaw = self.get_odom_data()
            if current_yaw is None:
                rospy.logwarn("Failed to get current orientation during rotation")
                # Continue trying for a bit in case it's just a temporary failure
                if time.time() - start_time > 5.0:  # Timeout after 5 seconds
                    rospy.logerr("Timeout waiting for odometry data during rotation")
                    return False
                self.rate.sleep()
                continue
                
            # Calculate remaining angle - difference between current and target in odometry frame
            angle_remaining = self.normalize_angle(target_yaw - current_yaw)
            
            # Check if we've reached the target
            if abs(angle_remaining) < self.angle_tolerance:
                rospy.loginfo(f"Reached target orientation: current={math.degrees(current_yaw):.1f}°, " +
                              f"target={math.degrees(target_yaw):.1f}°")
                break
                
            # Adaptive speed control - slow down as we approach target
            speed = min(self.angular_speed, max(0.1, abs(angle_remaining) * 0.8))
            
            # Set rotation direction based on shortest path
            if angle_remaining > 0:
                cmd.angular.z = speed
            else:
                cmd.angular.z = -speed
                
            # Add progress reporting every 2 seconds
            if time.time() - last_log_time > 2.0:
                rospy.loginfo(f"Rotation progress: current={math.degrees(current_yaw):.1f}°, " +
                              f"target={math.degrees(target_yaw):.1f}°, " +
                              f"remaining={math.degrees(angle_remaining):.1f}°")
                last_log_time = time.time()
                
            # Check for timeout (15 seconds max)
            if time.time() - start_time > 15.0:
                rospy.logerr(f"Rotation timeout reached. Current: {math.degrees(current_yaw):.1f}°, Target: {math.degrees(target_yaw):.1f}°")
                return False
                
            # Publish rotation command and sleep
            self.cmd_vel_pub.publish(cmd)
            self.rate.sleep()
            
        # Stop rotation
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)  # Brief pause
        return True
            
    def _move_straight(self, distance):
        """Move the robot straight forward by a specified distance using time-based control with feedback"""
        if distance < 0.05:  # Too small to move
            return
            
        rospy.loginfo(f"Moving forward {distance:.2f}m")
        
        # Get starting position for feedback
        start_x, start_y, _ = self.get_odom_data()
        if start_x is None:
            rospy.logwarn("Could not get starting position, using time-based movement instead")
            # Fall back to time-based movement
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            movement_time = distance / self.linear_speed
            
            # Move
            start_time = time.time()
            while time.time() - start_time < movement_time and not rospy.is_shutdown():
                self.cmd_vel_pub.publish(cmd)
                self.rate.sleep()
                
            # Stop
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.5)
            return
        
        # Move with odometry feedback
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        
        # Move
        start_time = time.time()
        last_log_time = start_time
        dist_moved = 0
        
        while dist_moved < distance and not rospy.is_shutdown():
            # Get current position
            curr_x, curr_y, _ = self.get_odom_data()
            if curr_x is not None and curr_y is not None:
                # Calculate distance moved
                dx = curr_x - start_x
                dy = curr_y - start_y
                dist_moved = math.sqrt(dx*dx + dy*dy)
                
                # Log progress periodically
                if time.time() - last_log_time > 2.0:
                    rospy.loginfo(f"Movement progress: {dist_moved:.2f}m / {distance:.2f}m ({dist_moved/distance*100:.1f}%)")
                    last_log_time = time.time()
                    
                # Adjust speed as we get closer (slow down)
                remaining = distance - dist_moved
                if remaining < 0.5:
                    cmd.linear.x = max(0.05, self.linear_speed * (remaining / 0.5))
            
            # Check for timeout (allow 3x the expected time)
            expected_time = distance / self.linear_speed
            if time.time() - start_time > expected_time * 3 + 10:
                rospy.logwarn(f"Movement timeout. Expected {expected_time:.1f}s, elapsed {time.time() - start_time:.1f}s")
                break
                
            self.cmd_vel_pub.publish(cmd)
            self.rate.sleep()
            
        # Stop
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)
        
    def rotate_in_place(self, angle):
        """
        Rotate the robot in place by the specified angle
        
        Args:
            angle: Angle to rotate in radians (positive=counterclockwise)
        """
        return self._rotate_to_angle(angle)