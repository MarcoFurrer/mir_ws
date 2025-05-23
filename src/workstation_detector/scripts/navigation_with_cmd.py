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
        self.linear_speed = 0.1  # m/s
        self.angular_speed = 0.2  # rad/s
        self.distance_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # radians (≈5.7°)
        
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
        rate = rospy.Rate(10)  # 10 Hz
        while (odom_data['x'] is None or odom_data['y'] is None or odom_data['yaw'] is None) and rospy.Time.now() < timeout:
            rate.sleep()
        
        # Unregister to avoid callback overhead
        odom_sub.unregister()
        
        return odom_data['x'], odom_data['y'], odom_data['yaw']

    def navigate_to_pose_with_cmd_vel(self, pose, frame_id='base_link'):
        """
        Navigate to a pose using direct velocity commands
        
        Args:
            pose: geometry_msgs/Pose target pose
            frame_id: Frame ID of the pose 
            
        Returns:
            True if navigation command was executed successfully
        """
        rospy.loginfo("Starting navigation to pose...")
        
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
        turn_cmd.angular.z = self.angular_speed if angle_to_target > 0 else -self.angular_speed
        
        # Turn for approximately the right amount of time
        turn_time = abs(angle_to_target) / abs(turn_cmd.angular.z)
        turn_time = min(turn_time, 5.0)  # Cap turn time at 5 seconds for safety
        
        rospy.loginfo(f"Turning for {turn_time:.1f} seconds to face angle {math.degrees(angle_to_target):.1f}°")
        
        # Publish turn command for the calculated duration
        start_time = time.time()
        rate = rospy.Rate(10)  # 10 Hz control loop
        while time.time() - start_time < turn_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(turn_cmd)
            rate.sleep()
        
        # Stop turning
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.sleep(1.0)  # Pause briefly to stabilize
        
        # Phase 2: Move forward toward the target
        drive_distance = distance_to_target
        rospy.loginfo(f"Phase 2: Moving forward {drive_distance:.2f} meters...")
        
        # Set forward speed
        forward_cmd = Twist()
        forward_cmd.linear.x = self.linear_speed
        drive_time = drive_distance / self.linear_speed
        drive_time = min(drive_time, 10.0)  # Cap at 10 seconds for safety
        
        # Publish forward command for the calculated duration
        start_time = time.time()
        while time.time() - start_time < drive_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(forward_cmd)
            rate.sleep()
        
        # Stop the robot
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Phase 3: Rotate to the final orientation
        final_rotation = yaw - angle_to_target
        rospy.loginfo(f"Phase 3: Rotating to final orientation {math.degrees(yaw):.1f}°...")
        
        # Set rotation speed and direction
        turn_cmd = Twist()
        turn_cmd.angular.z = self.angular_speed if final_rotation > 0 else -self.angular_speed
        
        # Turn for approximately the right amount of time
        turn_time = abs(final_rotation) / abs(turn_cmd.angular.z)
        turn_time = min(turn_time, 5.0)  # Cap turn time at 5 seconds for safety
        
        # Publish turn command for the calculated duration
        start_time = time.time()
        while time.time() - start_time < turn_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(turn_cmd)
            rate.sleep()
        
        # Stop the robot
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.loginfo("Navigation to pose completed")
        
        return True

    def navigate_to_pose_with_correction(self, pose, frame_id='base_link', max_attempts=3, distance_tolerance=None):
        """
        Navigate to a pose using odometry for correction
        
        Strategy:
        1. Get initial odom position
        2. Drive to target
        3. Get new odom position
        4. Calculate difference from expected
        5. Make additional movement to correct difference
        
        Args:
            pose: geometry_msgs/Pose target pose
            frame_id: Frame ID of the pose
            max_attempts: Maximum number of correction attempts
            distance_tolerance: Success threshold distance in meters
            
        Returns:
            True if navigation was successful
        """
        if distance_tolerance is None:
            distance_tolerance = self.distance_tolerance
            
        rospy.loginfo("Starting navigation with odometry correction...")
        
        # Get target position and orientation
        target_x = pose.position.x
        target_y = pose.position.y
        
        # Extract yaw angle from quaternion
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        _, _, target_yaw = euler_from_quaternion(quaternion)
        
        rospy.loginfo(f"Target pose: position=({target_x:.2f}, {target_y:.2f}), orientation={math.degrees(target_yaw):.1f}°")
        
        # Get initial position from odometry
        initial_x, initial_y, initial_yaw = self.get_odom_data()
        if initial_x is None or initial_y is None or initial_yaw is None:
            rospy.logerr("Failed to get initial odometry data")
            return False
        
        rospy.loginfo(f"Initial position from odometry: ({initial_x:.2f}, {initial_y:.2f}), orientation={math.degrees(initial_yaw):.1f}°")
        
        # Expected final position after movement
        expected_x = initial_x + target_x
        expected_y = initial_y + target_y
        
        # Expected final orientation (adding relative change to initial)
        expected_yaw = initial_yaw + target_yaw
        while expected_yaw > math.pi:
            expected_yaw -= 2 * math.pi
        while expected_yaw < -math.pi:
            expected_yaw += 2 * math.pi
            
        rospy.loginfo(f"Expected final position: ({expected_x:.2f}, {expected_y:.2f}), orientation={math.degrees(expected_yaw):.1f}°")
        
        # Attempt basic navigation
        self.navigate_to_pose_with_cmd_vel(pose, frame_id)
        
        # Get actual position after navigation
        actual_x, actual_y, actual_yaw = self.get_odom_data()
        if actual_x is None or actual_y is None or actual_yaw is None:
            rospy.logerr("Failed to get odometry data after navigation")
            return False
        
        rospy.loginfo(f"Actual position after navigation: ({actual_x:.2f}, {actual_y:.2f}), orientation={math.degrees(actual_yaw):.1f}°")
        
        # Calculate error
        error_x = expected_x - actual_x
        error_y = expected_y - actual_y
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        error_yaw = expected_yaw - actual_yaw
        # Normalize to [-pi, pi]
        while error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        while error_yaw < -math.pi:
            error_yaw += 2 * math.pi
        
        rospy.loginfo(f"Position error: ({error_x:.3f}, {error_y:.3f}) meters, distance={error_distance:.3f}m")
        rospy.loginfo(f"Orientation error: {math.degrees(error_yaw):.1f}°")
        
        # Check if correction is needed
        if error_distance <= distance_tolerance and abs(error_yaw) <= self.angle_tolerance:
            rospy.loginfo("Target reached within tolerance, no correction needed!")
            return True
        
        # Make corrections if needed
        attempts = 1
        while attempts < max_attempts and error_distance > distance_tolerance and not rospy.is_shutdown():
            attempts += 1
            rospy.loginfo(f"Making correction attempt {attempts}/{max_attempts}")
            
            # Create correction pose in robot's current frame
            correction_pose = Pose()
            
            # Convert correction vector to robot's local frame
            # We need to rotate the error vector by -yaw to get local coordinates
            local_error_x = error_x * math.cos(-actual_yaw) - error_y * math.sin(-actual_yaw)
            local_error_y = error_x * math.sin(-actual_yaw) + error_y * math.cos(-actual_yaw)
            
            correction_pose.position.x = local_error_x
            correction_pose.position.y = local_error_y
            
            # Set orientation to get to the right final heading
            correction_quat = quaternion_from_euler(0, 0, error_yaw)
            correction_pose.orientation.x = correction_quat[0]
            correction_pose.orientation.y = correction_quat[1]
            correction_pose.orientation.z = correction_quat[2]
            correction_pose.orientation.w = correction_quat[3]
            
            rospy.loginfo(f"Correction vector: ({local_error_x:.3f}, {local_error_y:.3f}) meters, orientation={math.degrees(error_yaw):.1f}°")
            
            # Execute correction
            self.navigate_to_pose_with_cmd_vel(correction_pose, 'base_link')
            
            # Get new position after correction
            actual_x, actual_y, actual_yaw = self.get_odom_data()
            if actual_x is None or actual_y is None or actual_yaw is None:
                rospy.logerr("Failed to get odometry data after correction")
                break
            
            # Recalculate error
            error_x = expected_x - actual_x
            error_y = expected_y - actual_y
            error_distance = math.sqrt(error_x**2 + error_y**2)
            
            error_yaw = expected_yaw - actual_yaw
            # Normalize to [-pi, pi]
            while error_yaw > math.pi:
                error_yaw -= 2 * math.pi
            while error_yaw < -math.pi:
                error_yaw += 2 * math.pi
            
            rospy.loginfo(f"Position after correction: ({actual_x:.2f}, {actual_y:.2f})")
            rospy.loginfo(f"Remaining error: distance={error_distance:.3f}m, orientation={math.degrees(error_yaw):.1f}°")
            
            # Check if we're close enough now
            if error_distance <= distance_tolerance:
                rospy.loginfo("Target position reached within tolerance!")
                
                # Final orientation correction if needed
                if abs(error_yaw) > self.angle_tolerance:
                    rospy.loginfo(f"Making final orientation correction of {math.degrees(error_yaw):.1f}°")
                    
                    # Rotate in place
                    turn_cmd = Twist()
                    turn_cmd.angular.z = self.angular_speed if error_yaw > 0 else -self.angular_speed
                    
                    # Calculate turn time
                    turn_time = abs(error_yaw) / abs(turn_cmd.angular.z)
                    turn_time = min(turn_time, 5.0)  # Cap at 5 seconds
                    
                    rospy.loginfo(f"Rotating for {turn_time:.1f} seconds")
                    
                    # Execute turn
                    start_time = time.time()
                    rate = rospy.Rate(10)
                    while time.time() - start_time < turn_time and not rospy.is_shutdown():
                        self.cmd_vel_pub.publish(turn_cmd)
                        rate.sleep()
                    
                    # Stop rotation
                    self.cmd_vel_pub.publish(Twist())
                
                return True
        
        if error_distance <= distance_tolerance:
            rospy.loginfo("Navigation succeeded with corrections!")
            return True
        else:
            rospy.logwarn(f"Navigation didn't reach target within tolerance after {max_attempts} attempts")
            rospy.logwarn(f"Final error: distance={error_distance:.3f}m")
            return False
    
    def rotate_in_place(self, angle):
        """
        Rotate the robot in place by a specific angle
        
        Args:
            angle: Angle to rotate in radians (positive=counterclockwise)
        """
        # Set rotation speed and direction
        turn_cmd = Twist()
        turn_cmd.angular.z = self.angular_speed if angle > 0 else -self.angular_speed
        
        # Calculate turn time
        turn_time = abs(angle) / abs(turn_cmd.angular.z)
        turn_time = min(turn_time, 5.0)  # Cap at 5 seconds for safety
        
        # Execute turn
        rospy.loginfo(f"Rotating for {turn_time:.1f}s at {math.degrees(turn_cmd.angular.z):.1f}°/s")
        start_time = time.time()
        rate = rospy.Rate(10)
        while time.time() - start_time < turn_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(turn_cmd)
            rate.sleep()
        
        # Stop rotation
        self.cmd_vel_pub.publish(Twist())


def main():
    """Main function to demonstrate the navigation controller"""
    rospy.init_node('navigation_controller')
    rospy.loginfo("Starting navigation controller node...")
    
    controller = NavigationController()
    
    # Test odometry reading
    x, y, yaw = controller.get_odom_data()
    if x is not None and y is not None and yaw is not None:
        rospy.loginfo(f"Current position: ({x:.2f}, {y:.2f}), orientation: {math.degrees(yaw):.1f}°")
    else:
        rospy.logerr("Failed to get odometry data!")
        return
    
    # Ask the user if they want to test navigation
    rospy.loginfo("Navigation controller initialized.")
    rospy.loginfo("Press Ctrl+C to exit or wait 5 seconds to continue with a navigation test...")
    try:
        rospy.sleep(5.0)
    except rospy.ROSInterruptException:
        return
    
    # Create a simple test pose (move 1 meter forward)
    test_pose = Pose()
    test_pose.position.x = 1.0
    test_pose.position.y = 0.0
    test_pose.position.z = 0.0
    
    # Set orientation to keep current heading
    quat = quaternion_from_euler(0, 0, 0)
    test_pose.orientation.x = quat[0]
    test_pose.orientation.y = quat[1]
    test_pose.orientation.z = quat[2]
    test_pose.orientation.w = quat[3]
    
    # Navigate using the correction-based method
    rospy.loginfo("Testing navigation with correction...")
    result = controller.navigate_to_pose_with_correction(test_pose)
    
    if result:
        rospy.loginfo("Navigation test completed successfully!")
    else:
        rospy.logwarn("Navigation test failed!")
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
