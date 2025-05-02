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
        self.processing_interval = 1.0 
        
        
        
        # Subscribers - try multiple possible scan topics
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Store detected workstations
        self.workstations = []
        
        rospy.loginfo("Workstation detector initialized")
        rospy.loginfo("Processing scans every %.1f seconds", self.processing_interval)

    def laser_callback(self, msg):
        """Process laser scan data at a controlled rate"""
        current_time = time.time()
        
        # Only process scans at the specified interval
        if current_time - self.last_processed_time < self.processing_interval:
            return
            
        self.last_processed_time = current_time
        
        # Extract valid scan points
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates (robot frame)
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        rospy.loginfo("\n--- Processing scan with %d valid points from %s ---", 
                     len(valid_ranges), msg.header.frame_id)
        
        # Basic scan statistics
        if len(valid_ranges) > 0:
            min_dist = min(valid_ranges)
            max_dist = max(valid_ranges)
            avg_dist = sum(valid_ranges) / len(valid_ranges)
            rospy.loginfo("Scan statistics: min=%.2fm, max=%.2fm, avg=%.2fm",
                         min_dist, max_dist, avg_dist)
        
        # Find clusters of points that could be objects
        clusters = self.cluster_scan_points(x_points, y_points)
        
        # Analyze each cluster to see if it matches workstation dimensions
        self.analyze_clusters(clusters, msg.header.frame_id)


if __name__ == '__main__':
    try:
        detector = WorkstationDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass