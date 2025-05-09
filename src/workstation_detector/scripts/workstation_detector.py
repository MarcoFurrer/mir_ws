#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time

class WorkstationDetector:
    def __init__(self):
        rospy.init_node('workstation_detector')
        
        # Subscribers - try multiple possible scan topics
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
    
    def laser_callback(self, msg):
        """
        Process laser scan data and return a list of (x,y) points
        """
        # Simple print to verify callback is being called
        print("Laser callback triggered!")
        
        # Get the points from the laser scan
        points = self.laser_scan_to_points(msg)
        
        # Print basic information about the points
        print(f"Extracted {len(points)} points from laser scan")
        
        # Optionally print the first few points for debugging
        if points:
            print(f"First 5 points: {points[:5]}")
        
        return points
    
    def laser_scan_to_points(self, scan_msg):
        """
        Convert a LaserScan message to a list of (x,y) points
        """
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))
        
        # Filter out invalid readings (infinity, NaN, etc.)
        valid_indices = np.isfinite(ranges)
        
        # Apply range limits if available
        if hasattr(scan_msg, 'range_min') and hasattr(scan_msg, 'range_max'):
            range_filter = np.logical_and(ranges >= scan_msg.range_min, 
                                         ranges <= scan_msg.range_max)
            valid_indices = np.logical_and(valid_indices, range_filter)
        
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert polar (range, angle) to Cartesian (x, y) coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        # Combine into a list of (x,y) points
        points = list(zip(x_points, y_points))
        
        return points


if __name__ == '__main__':
    try:
        detector = WorkstationDetector()
        print("Workstation detector running - waiting for laser scan data...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass