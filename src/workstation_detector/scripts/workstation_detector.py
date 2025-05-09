#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time
import cv2
import matplotlib.pyplot as plt

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
        
        # Add marker publisher for RViz visualization
        self.marker_pub = rospy.Publisher('detected_lines', MarkerArray, queue_size=10)
        
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
            
            # Detect lines using Hough transform
            lines = self.detect_lines_hough(points, frame_id=msg.header.frame_id)
            if lines:
                rospy.loginfo(f"Detected {len(lines)} lines in the scan")

    def detect_lines_hough(self, points, rho=0.05, theta=np.pi/180, 
                       threshold=10, min_line_length=0.3, max_line_gap=0.2, frame_id='base_link'):
        """
        Detect lines in laser scan data using Hough Transform
        
        Args:
            points: List of (x, y) points
            rho: Distance resolution of the accumulator in meters
            theta: Angle resolution of the accumulator in radians
            threshold: Minimum number of intersections to detect a line
            min_line_length: Minimum line length in meters
            max_line_gap: Maximum gap between points on the same line in meters
            frame_id: Frame ID of the laser scan for visualization
            
        Returns:
            List of detected lines as (rho, theta) pairs
        """
        if not points or len(points) < threshold:
            print("Not enough points for line detection")
            return []
        
        # Convert points to numpy array
        points_array = np.array(points)
        
        # Find min and max coordinates to determine scale
        min_x, min_y = np.min(points_array, axis=0) - 0.5
        max_x, max_y = np.max(points_array, axis=0) + 0.5
        
        # Define scaling to convert from meters to pixels
        # Use a reasonable resolution (1 pixel = 1cm)
        pixels_per_meter = 100
        img_width = int((max_x - min_x) * pixels_per_meter)
        img_height = int((max_y - min_y) * pixels_per_meter)
        
        # Create blank image
        img = np.zeros((img_height, img_width), dtype=np.uint8)
        
        # Map points to image coordinates
        for x, y in points:
            px = int((x - min_x) * pixels_per_meter)
            py = int((y - min_y) * pixels_per_meter)
            
            # Ensure point is within image bounds
            if 0 <= px < img_width and 0 <= py < img_height:
                img[py, px] = 255
        
        # Apply Hough Transform
        lines = cv2.HoughLinesP(
            img, 
            rho=rho * pixels_per_meter,
            theta=theta,
            threshold=threshold,
            minLineLength=int(min_line_length * pixels_per_meter),
            maxLineGap=int(max_line_gap * pixels_per_meter)
        )
        
        detected_lines = []
        
        if lines is not None:
            print(f"Hough Transform detected {len(lines)} line segments")
            
            # Process each line
            for line in lines:
                x1, y1, x2, y2 = line[0]
                
                # Convert back to meters
                x1_m = (x1 / pixels_per_meter) + min_x
                y1_m = (y1 / pixels_per_meter) + min_y
                x2_m = (x2 / pixels_per_meter) + min_x
                y2_m = (y2 / pixels_per_meter) + min_y
                
                # Calculate line length
                length = np.sqrt((x2_m - x1_m)**2 + (y2_m - y1_m)**2)
                
                # Calculate line angle (in degrees)
                angle = np.degrees(np.arctan2(y2_m - y1_m, x2_m - x1_m))
                
                # Store line parameters
                detected_line = {
                    'start': (x1_m, y1_m),
                    'end': (x2_m, y2_m),
                    'length': length,
                    'angle': angle
                }
                
                detected_lines.append(detected_line)
                print(f"Line detected: length={length:.2f}m, angle={angle:.1f}°")
            
            # Publish markers for RViz visualization
            self.publish_line_markers(detected_lines, frame_id)
        
        return detected_lines

    def publish_line_markers(self, lines, frame_id):
        """
        Publish detected lines as RViz markers
        
        Args:
            lines: List of detected lines
            frame_id: Frame ID from the laser scan
        """
        marker_array = MarkerArray()
        
        # Use current time for all markers to prevent "too old" errors
        current_time = rospy.Time.now()
        
        # Delete previous markers
        delete_marker = Marker()
        delete_marker.header.stamp = current_time
        delete_marker.header.frame_id = frame_id
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Create a marker for each line
        for i, line in enumerate(lines):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = current_time  # Use current time for all markers
            marker.ns = "detected_lines"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Add a lifetime to the marker
            marker.lifetime = rospy.Duration(1.0)  # Display for 1 second
            
            # Set line properties
            marker.scale.x = 0.05  # Line width
            
            # Color based on line angle for better visualization
            angle = line['angle'] % 180
            # Use HSV color space for smooth color transitions
            # Map angle 0-180 to hue 0-1
            hue = angle / 180.0
            # Convert to RGB (simplified conversion)
            if hue < 1/6:
                marker.color.r = 1.0
                marker.color.g = hue * 6
                marker.color.b = 0.0
            elif hue < 2/6:
                marker.color.r = (2/6 - hue) * 6
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif hue < 3/6:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = (hue - 2/6) * 6
            elif hue < 4/6:
                marker.color.r = 0.0
                marker.color.g = (4/6 - hue) * 6
                marker.color.b = 1.0
            elif hue < 5/6:
                marker.color.r = (hue - 4/6) * 6
                marker.color.g = 0.0
                marker.color.b = 1.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = (1.0 - hue) * 6
                
            marker.color.a = 1.0  # Full opacity
            
            # Set line endpoints
            start_point = Point()
            start_point.x = line['start'][0]
            start_point.y = line['start'][1]
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = line['end'][0]
            end_point.y = line['end'][1]
            end_point.z = 0.0
            
            marker.points.append(start_point)
            marker.points.append(end_point)
            
            # Add to marker array
            marker_array.markers.append(marker)
            
        # Publish the marker array
        self.marker_pub.publish(marker_array)
        print(f"Published {len(lines)} line markers to RViz")

    def process_laser_scan(self, msg):
        """
        Process the laser scan message and return valid points
        """
        print("Processing laser scan...")
        # Extract ranges and angles
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter out invalid readings
        valid_indices = np.logical_and(
            ranges > msg.range_min, 
            ranges < msg.range_max
        )
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Convert to Cartesian coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        print(f"Found {len(x_points)} valid points")
        return list(zip(x_points, y_points))


if __name__ == '__main__':
    try:
        detector = WorkstationDetector()
        rospy.loginfo("Workstation detector running. Press Ctrl+C to stop.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass