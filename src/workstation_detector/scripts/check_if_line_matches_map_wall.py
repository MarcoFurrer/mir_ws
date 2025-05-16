#!/usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import time
import cv2
import matplotlib.pyplot as plt


def check_if_line_matches_map_wall(self, line, map_data):
    """
    Check if a line matches a wall in the map
    
    Args:
        line: Line to check
        map_data: OccupancyGrid map data
        
    Returns:
        True if line is likely a wall, False otherwise
    """
    try:
        # Transform line from laser frame to map frame
        transform = self.tf_buffer.lookup_transform(
            'map',
            self.latest_frame_id,
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        
        # Sample points along the line
        num_samples = 10
        points = []
        for i in range(num_samples):
            alpha = i / (num_samples - 1)
            x = line['start'][0] + alpha * (line['end'][0] - line['start'][0])
            y = line['start'][1] + alpha * (line['end'][1] - line['start'][1])
            
            # Create stamped point
            p = geometry_msgs.msg.PointStamped()
            p.header.frame_id = self.latest_frame_id
            p.point.x = x
            p.point.y = y
            p.point.z = 0.0
            
            # Transform to map frame
            p_map = tf2_ros.do_transform_point(p, transform)
            points.append((p_map.point.x, p_map.point.y))
        
        # Check if points match occupied cells in the map
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        width = map_data.info.width
        
        wall_match_count = 0
        
        for px, py in points:
            # Convert to grid coordinates
            grid_x = int((px - origin_x) / resolution)
            grid_y = int((py - origin_y) / resolution)
            
            # Check if in bounds
            if 0 <= grid_x < width and 0 <= grid_y < map_data.info.height:
                # Get cell value (100 = occupied, 0 = free, -1 = unknown)
                index = grid_y * width + grid_x
                cell_value = map_data.data[index]
                
                # If occupied cell, increment counter
                if cell_value > 50:  # Occupied
                    wall_match_count += 1
                    
        # If more than 60% of points match walls in the map, it's likely a wall
        return wall_match_count > 0.6 * num_samples
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
            tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Could not transform line to map frame: {e}")
        return False