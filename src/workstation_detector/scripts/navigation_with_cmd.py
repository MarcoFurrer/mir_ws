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
        self.odom_initial_position = self.get_odom_data()
        
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
        Navigate to a pose using odometry delta tracking approach
        
        Args:
            pose: geometry_msgs/Pose target pose in robot's frame
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
        
        rospy.loginfo(f"Target: position=({target_x:.2f}, {target_y:.2f}), orientation={math.degrees(target_yaw):.1f}°")
        
        for attempt in range(1, max_attempts + 1):
            if attempt > 1:
                rospy.loginfo(f"Starting correction attempt {attempt}/{max_attempts}")
        
            # Get reference position at the start of this attempt
            ref_x, ref_y, ref_yaw = self.get_odom_data()
            if ref_x is None:
                rospy.logerr("Failed to get odometry data. Navigation aborted.")
                return False
                
            rospy.loginfo(f"Reference position: ({ref_x:.2f}, {ref_y:.2f}), orientation={math.degrees(ref_yaw):.1f}°")
            
            # 1. Rotate to face target
            angle_to_target = math.atan2(target_y, target_x)
            self._rotate_to_angle(angle_to_target)
            
            # 2. Move straight to target position
            distance = math.sqrt(target_x**2 + target_y**2)
            self._move_straight(distance)
            
            # 3. Rotate to final orientation
            self._rotate_to_angle(target_yaw)
            
            # Check how far we've moved from reference point
            current_x, current_y, current_yaw = self.get_odom_data()
            if current_x is None:
                rospy.logwarn("Failed to get position after navigation")
                continue
                
            # Calculate delta movement in odometry frame
            delta_x = current_x - ref_x
            delta_y = current_y - ref_y
            
            # Convert delta to robot's initial frame
            # We need to rotate by -ref_yaw to convert from odom to robot's initial frame
            rotated_delta_x = delta_x * math.cos(-ref_yaw) - delta_y * math.sin(-ref_yaw)
            rotated_delta_y = delta_x * math.sin(-ref_yaw) + delta_y * math.cos(-ref_yaw)
            
            # Calculate error between what we wanted and what we got
            error_x = target_x - rotated_delta_x
            error_y = target_y - rotated_delta_y
            error_distance = math.sqrt(error_x**2 + error_y**2)
            
            rospy.loginfo(f"Movement delta: ({rotated_delta_x:.2f}, {rotated_delta_y:.2f})")
            rospy.loginfo(f"Position error: {error_distance:.2f}m")
            
            # If error is acceptable or this was the last attempt, we're done
            if error_distance <= self.distance_tolerance or attempt >= max_attempts:
                break
                
            # Update target for next correction attempt
            target_x = error_x
            target_y = error_y
            
            rospy.loginfo(f"Correction vector: ({error_x:.2f}, {error_y:.2f})")
    
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