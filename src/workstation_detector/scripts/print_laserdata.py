#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def laser_callback(msg):
    """Process incoming laser scan data"""
    # Print basic information about the scan
    rospy.loginfo("Received laser scan with %d points", len(msg.ranges))
    
    # Calculate statistics of the scan
    valid_ranges = [r for r in msg.ranges if r > msg.range_min and r < msg.range_max]
    if valid_ranges:
        min_dist = min(valid_ranges)
        max_dist = max(valid_ranges)
        avg_dist = sum(valid_ranges) / len(valid_ranges)
        
        rospy.loginfo("Scan statistics: min=%.2fm, max=%.2fm, avg=%.2fm", 
                    min_dist, max_dist, avg_dist)
        
        # Check for close obstacles (less than 0.5 meters)
        close_points = [r for r in valid_ranges if r < 0.5]
        if close_points:
            rospy.loginfo("WARNING: %d close obstacles detected! Nearest: %.2fm", 
                        len(close_points), min(close_points))
    else:
        rospy.loginfo("No valid range readings in this scan")

def main():
    """Initialize the node and subscribers"""
    rospy.init_node('laser_data_printer')
    
    # Try both front and back scanner topics (MiR robots often have both)
    rospy.Subscriber('/f_scan', LaserScan, laser_callback)
    rospy.Subscriber('/b_scan', LaserScan, laser_callback)
    
    # Also try the standard ROS topic name as a fallback
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    
    rospy.loginfo("Laser data printer initialized. Waiting for laser scan messages...")
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass