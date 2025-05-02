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
        
        # Target dimensions for workstations
        self.target_length = 0.70  # 70cm
        self.target_width = 0.35   # 35cm
        
        # Tolerance for dimension matching (percentage)
        self.dimension_tolerance = 0.25  # 25% tolerance
        
        # Parameters for clustering
        self.max_gap = 0.1  # Maximum gap between points in the same cluster (10cm)
        self.min_points = 8  # Minimum number of points to form a cluster
        self.min_cluster_size = 0.25  # Minimum size in meters to consider a cluster
        
        # Processing rate control
        self.processing_interval = 1.0  # Process a scan every second
        self.last_processed_time = 0
        
        # Publishers
        self.marker_pub = rospy.Publisher('/workstation_markers', MarkerArray, queue_size=10)
        self.cluster_pub = rospy.Publisher('/laser_clusters', MarkerArray, queue_size=10)
        
        # Subscribers - try multiple possible scan topics
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Store detected workstations
        self.workstations = []
        
        rospy.loginfo("Workstation detector initialized")
        rospy.loginfo("Looking for objects approximately %.2fm x %.2fm", 
                     self.target_length, self.target_width)
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

    def cluster_scan_points(self, x_points, y_points):
        """Group scan points into clusters based on proximity"""
        if len(x_points) == 0:
            return []
            
        # Convert to list of points
        points = list(zip(x_points, y_points))
        
        # Sort points by angle around robot (atan2) for sequential processing
        points.sort(key=lambda p: math.atan2(p[1], p[0]))
        
        clusters = []
        current_cluster = [points[0]]
        
        # Group points that are close to each other
        for i in range(1, len(points)):
            prev_point = points[i-1]
            curr_point = points[i]
            
            # Calculate distance between consecutive points
            distance = math.sqrt((curr_point[0] - prev_point[0])**2 + 
                                (curr_point[1] - prev_point[1])**2)
            
            if distance <= self.max_gap:
                # Add to current cluster
                current_cluster.append(curr_point)
            else:
                # Start a new cluster if gap is too large
                if len(current_cluster) >= self.min_points:
                    clusters.append(current_cluster)
                current_cluster = [curr_point]
        
        # Add the last cluster if it has enough points
        if len(current_cluster) >= self.min_points:
            clusters.append(current_cluster)
            
        rospy.loginfo("Found %d initial clusters", len(clusters))
        return clusters
        
    def analyze_clusters(self, clusters, frame_id):
        """Check each cluster to see if it matches workstation dimensions"""
        workstation_markers = MarkerArray()
        cluster_markers = MarkerArray()
        marker_id = 0
        
        # Filter for larger clusters
        large_clusters = []
        
        for cluster in clusters:
            # Extract x and y coordinates
            x_coords = [p[0] for p in cluster]
            y_coords = [p[1] for p in cluster]
            
            # Calculate basic dimensions using bounding box
            min_x, max_x = min(x_coords), max(x_coords)
            min_y, max_y = min(y_coords), max(y_coords)
            
            width = max_x - min_x
            height = max_y - min_y
            
            # Calculate center point
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            
            # Calculate distance from robot
            distance = math.sqrt(center_x**2 + center_y**2)
            
            # Only consider clusters bigger than minimum size
            if width > self.min_cluster_size and height > self.min_cluster_size:
                large_clusters.append({
                    'cluster': cluster, 
                    'width': width, 
                    'height': height,
                    'center_x': center_x,
                    'center_y': center_y,
                    'distance': distance,
                    'bearing': math.degrees(math.atan2(center_y, center_x))
                })
                
                # Add cluster visualization
                cluster_marker = Marker()
                cluster_marker.header.frame_id = frame_id
                cluster_marker.header.stamp = rospy.Time.now()
                cluster_marker.ns = "laser_clusters"
                cluster_marker.id = marker_id
                marker_id += 1
                cluster_marker.type = Marker.LINE_STRIP
                cluster_marker.action = Marker.ADD
                cluster_marker.pose.orientation.w = 1.0
                cluster_marker.scale.x = 0.02  # Line width
                
                # Color by size - larger objects are more red
                size_ratio = min(1.0, max(width, height) / max(self.target_length, self.target_width))
                cluster_marker.color.r = 0.5 + size_ratio * 0.5  # More red for bigger clusters
                cluster_marker.color.g = 0.8 - size_ratio * 0.5  # Less green for bigger clusters
                cluster_marker.color.b = 0.2
                cluster_marker.color.a = 0.8
                
                cluster_marker.lifetime = rospy.Duration(1.5)  # 1.5 seconds
                
                # Add points to create outline
                for point in cluster:
                    p = Point()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = 0.05
                    cluster_marker.points.append(p)
                    
                # Close the loop
                if len(cluster) > 0:
                    p = Point()
                    p.x = cluster[0][0]
                    p.y = cluster[0][1]
                    p.z = 0.05
                    cluster_marker.points.append(p)
                    
                cluster_markers.markers.append(cluster_marker)
        
        # Print info about large clusters
        if large_clusters:
            rospy.loginfo("Found %d large clusters:", len(large_clusters))
            for i, c in enumerate(large_clusters):
                rospy.loginfo("  Cluster #%d: %.2fm × %.2fm at distance %.2fm, bearing %.1f°", 
                             i+1, c['width'], c['height'], c['distance'], c['bearing'])
                
                # Check if dimensions approximately match our target size (in either orientation)
                tolerance = self.dimension_tolerance
                is_match = False
                orientation = 0
                
                # Check if matches expected dimensions in either orientation
                if (abs(c['width'] - self.target_length) <= self.target_length * tolerance and 
                    abs(c['height'] - self.target_width) <= self.target_width * tolerance):
                    # Machine is horizontal (length along x-axis)
                    orientation = 0
                    is_match = True
                    rospy.loginfo("    ** MATCH: Workstation detected! (horizontal orientation)")
                elif (abs(c['width'] - self.target_width) <= self.target_width * tolerance and 
                      abs(c['height'] - self.target_length) <= self.target_length * tolerance):
                    # Machine is vertical (length along y-axis)
                    orientation = math.pi / 2
                    is_match = True
                    rospy.loginfo("    ** MATCH: Workstation detected! (vertical orientation)")
                
                # Also accept partial views if dimensions are reasonable
                if not is_match:
                    if ((c['width'] > self.target_width * 1.2 and c['width'] < self.target_length and
                         c['height'] > self.target_width * 0.7) or
                        (c['height'] > self.target_width * 1.2 and c['height'] < self.target_length and
                         c['width'] > self.target_width * 0.7)):
                        orientation = 0 if c['width'] > c['height'] else math.pi / 2
                        is_match = True
                        rospy.loginfo("    ** PARTIAL MATCH: Possible partial view of workstation")
                
                if is_match:
                    # Create workstation marker
                    ws_marker = Marker()
                    ws_marker.header.frame_id = frame_id
                    ws_marker.header.stamp = rospy.Time.now()
                    ws_marker.ns = "workstations"
                    ws_marker.id = i
                    ws_marker.type = Marker.CUBE
                    ws_marker.action = Marker.ADD
                    
                    # Set position and orientation
                    ws_marker.pose.position.x = c['center_x']
                    ws_marker.pose.position.y = c['center_y']
                    ws_marker.pose.position.z = 0.175  # Half height
                    
                    # Set orientation
                    if orientation == 0:  # horizontal
                        ws_marker.pose.orientation.w = 1.0
                    else:  # vertical
                        from tf.transformations import quaternion_from_euler
                        q = quaternion_from_euler(0, 0, orientation)
                        ws_marker.pose.orientation.x = q[0]
                        ws_marker.pose.orientation.y = q[1]
                        ws_marker.pose.orientation.z = q[2]
                        ws_marker.pose.orientation.w = q[3]
                    
                    # Set dimensions
                    ws_marker.scale.x = self.target_length
                    ws_marker.scale.y = self.target_width
                    ws_marker.scale.z = 0.35  # Height
                    
                    # Set color (green for match)
                    ws_marker.color.r = 0.0
                    ws_marker.color.g = 0.8
                    ws_marker.color.b = 0.2
                    ws_marker.color.a = 0.6
                    
                    ws_marker.lifetime = rospy.Duration(2.0)  # 2 seconds
                    
                    workstation_markers.markers.append(ws_marker)
        else:
            rospy.loginfo("No large clusters found in this scan")
        
        # Publish markers
        self.cluster_pub.publish(cluster_markers)
        self.marker_pub.publish(workstation_markers)

if __name__ == '__main__':
    try:
        detector = WorkstationDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass