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
        # Fix: Rate not rate (capital R)
        self.rate = rospy.Rate(1)
        self.processing_interval = 1.0
        self.last_processed_time = 0
        
        # Add debug flag for development
        self.debug = True
        print("Debug mode enabled - will print laser callback info")
        
        # Subscribers - try multiple possible scan topics
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Store detected workstations
        self.workstations = []
        
        rospy.loginfo("Workstation detector initialized")
        rospy.loginfo("Processing scans every %.1f seconds", self.processing_interval)

    def laser_callback(self, msg):
        """Process laser scan data at a controlled rate"""
        # Simple print to verify callback is being called
        print("Laser callback triggered!")
        
        current_time = time.time()
        
        # Only process scans at the specified interval
        if current_time - self.last_processed_time < self.processing_interval:
            return
            
        self.last_processed_time = current_time
        
        print(f"Processing scan message from topic: {msg._connection_header['topic']}")
        print(f"Scan contains {len(msg.ranges)} points")
        
        # Process the laser scan to get points
        points = self.process_laser_scan(msg)
        
        rospy.loginfo("\n--- Processing scan with %d valid points from %s ---", 
                     len(points), msg.header.frame_id)
        
        # Basic scan statistics
        if points:
            distances = [math.sqrt(p[0]**2 + p[1]**2) for p in points]
            min_dist = min(distances)
            max_dist = max(distances)
            avg_dist = sum(distances) / len(distances)
            rospy.loginfo("Scan statistics: min=%.2fm, max=%.2fm, avg=%.2fm",
                         min_dist, max_dist, avg_dist)
        


if __name__ == '__main__':
    try:
        detector = WorkstationDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass