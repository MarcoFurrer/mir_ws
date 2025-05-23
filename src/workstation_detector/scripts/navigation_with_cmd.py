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
        
        # Unregister to avoid callback overhead
        odom_sub.unregister()
        
        return odom_data['x'], odom_data['y'], odom_data['yaw']

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def navigate_to_pose(self, pose, frame_id='base_link', max_attempts=2):
        """
        Navigate to a pose using a simple and robust approach
        
        Args:
            pose: geometry_msgs/Pose target pose in robot's frame
            frame_id: Frame ID of the pose
            max_attempts: Maximum number of correction attempts
            
        Returns:
            True if navigation was successful
        """
        rospy.loginfo("Starting simple navigation to pose...")
        
        # Extract target information
        target_x = pose.position.x
        target_y = pose.position.y
        
        # Extract orientation as yaw angle
        quat = [pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w]
        _, _, target_yaw = euler_from_quaternion(quat)
        
        rospy.loginfo(f"Target: position=({target_x:.2f}, {target_y:.2f}), orientation={math.degrees(target_yaw):.1f}°")
        
        # Get initial position
        initial_x, initial_y, initial_yaw = self.get_odom_data()
        if initial_x is None:
            rospy.logerr("Failed to get odometry data. Navigation aborted.")
            return False
            
        rospy.loginfo(f"Initial position: ({initial_x:.2f}, {initial_y:.2f}), orientation={math.degrees(initial_yaw):.1f}°")
        
        # Transform target from robot's local frame to odometry frame
        rotated_x = target_x * math.cos(initial_yaw) - target_y * math.sin(initial_yaw)
        rotated_y = target_x * math.sin(initial_yaw) + target_y * math.cos(initial_yaw)
        
        # Expected final position in odometry frame
        expected_x = initial_x + rotated_x
        expected_y = initial_y + rotated_y
        expected_yaw = self.normalize_angle(initial_yaw + target_yaw)
        
        rospy.loginfo(f"Expected final position: ({expected_x:.2f}, {expected_y:.2f}), orientation={math.degrees(expected_yaw):.1f}°")
        
        # Simple navigation approach: rotate, move straight, rotate to final orientation
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                rospy.loginfo(f"Starting correction attempt {attempt}/{max_attempts}")
                
            # 1. Rotate to face target
            angle_to_target = math.atan2(target_y, target_x)
            self._rotate_to_angle(angle_to_target)
            
            # 2. Move straight to target position
            distance = math.sqrt(target_x**2 + target_y**2)
            self._move_straight(distance)
            
            # 3. Rotate to final orientation
            self._rotate_to_angle(target_yaw)
            
            # Check if we reached the target
            actual_x, actual_y, actual_yaw = self.get_odom_data()
            if actual_x is None:
                rospy.logwarn("Failed to get position after navigation")
                continue
                
            # Calculate error in odometry frame
            error_x = expected_x - actual_x
            error_y = expected_y - actual_y
            error_distance = math.sqrt(error_x**2 + error_y**2)
            
            rospy.loginfo(f"Position error: {error_distance:.2f}m")
            
            # If error is acceptable or this was the last attempt, we're done
            if error_distance <= self.distance_tolerance or attempt >= max_attempts:
                break
                
            # Calculate correction in robot's local frame for next attempt
            # Rotate the error vector by -current_yaw to get local coordinates
            correction_x = error_x * math.cos(-actual_yaw) - error_y * math.sin(-actual_yaw)
            correction_y = error_x * math.sin(-actual_yaw) + error_y * math.cos(-actual_yaw)
            
            # Update target for next correction attempt
            target_x = correction_x
            target_y = correction_y
            
            rospy.loginfo(f"Correction vector: ({correction_x:.2f}, {correction_y:.2f})")
            
        rospy.loginfo("Navigation complete")
        return True
        
    def _rotate_to_angle(self, target_angle):
        """Rotate the robot to face a specific angle in the robot's local frame"""
        rospy.loginfo(f"Rotating to angle {math.degrees(target_angle):.1f}°")
        
        # Get initial orientation
        _, _, initial_yaw = self.get_odom_data()
        if initial_yaw is None:
            rospy.logwarn("Failed to get orientation for rotation")
            return
            
        # Calculate how much to rotate
        cmd = Twist()
        
        # Set rotation direction
        if target_angle > 0:
            cmd.angular.z = self.angular_speed
        else:
            cmd.angular.z = -self.angular_speed
            
        # Start rotation
        rotation_time = abs(target_angle) / self.angular_speed
        start_time = time.time()
        
        while time.time() - start_time < rotation_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(cmd)
            self.rate.sleep()
            
        # Stop rotation
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)  # Brief pause
        
    def _move_straight(self, distance):
        """Move the robot straight forward by a specified distance"""
        if distance < 0.05:  # Too small to move
            return
            
        rospy.loginfo(f"Moving forward {distance:.2f}m")
        
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        
        # Calculate movement time
        movement_time = distance / self.linear_speed
        
        # Move
        start_time = time.time()
        while time.time() - start_time < movement_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(cmd)
            self.rate.sleep()
            
        # Stop
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)  # Brief pause
        
    def rotate_in_place(self, angle):
        """
        Rotate the robot in place by the specified angle
        
        Args:
            angle: Angle to rotate in radians (positive=counterclockwise)
        """
        self._rotate_to_angle(angle)
        return True