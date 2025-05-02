#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import tf
import math

class WorkstationDetector:
    def __init__(self):
        rospy.init_node('workstation_detector', anonymous=True)
        
        # Store workstation locations and count
        self.workstations_detected = []
        self.workstations_count = 0
        
        # Define machine dimensions (in meters)
        self.machine_length = 0.70  # 70cm - adjusted from 0.80
        self.machine_width = 0.35   # 35cm - adjusted from 0.30
        
        # Publishers
        self.workstation_pub = rospy.Publisher('/detected_workstations', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/workstation_markers', MarkerArray, queue_size=10)
        self.debug_marker_pub = rospy.Publisher('/candidate_points', MarkerArray, queue_size=10)
        
        # Subscribers
        self.static_map_sub = rospy.Subscriber('/map', OccupancyGrid, self.static_map_callback)
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        
        # Store maps
        self.static_map = None
        self.static_map_data = None
        self.costmap = None
        
        # Debug counters
        self.map_updates = 0
        
        rospy.loginfo("Machine detector initialized - looking for objects 70cm x 35cm")
    
    def static_map_callback(self, map_msg):
        self.static_map = map_msg
        self.static_map_data = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
        rospy.loginfo(f"Static map received: {map_msg.info.width}x{map_msg.info.height}, resolution: {map_msg.info.resolution}")
        
        # Print some statistics about the static map
        occupied = np.sum(self.static_map_data > 0)
        free = np.sum(self.static_map_data == 0)
        unknown = np.sum(self.static_map_data == -1)
        rospy.loginfo(f"Static map stats: {occupied} occupied cells, {free} free cells, {unknown} unknown cells")
    
    def costmap_callback(self, costmap_msg):
        if self.static_map is None:
            rospy.logwarn("Received costmap but static map not available yet")
            return
        
        self.map_updates += 1
        self.costmap = costmap_msg
        
        # Only process every 5th update to reduce computational load
        if self.map_updates % 5 == 0:
            rospy.loginfo("Processing costmap to find workstations")
            self.find_workstations()
    
    def find_workstations(self):
        if self.static_map is None or self.costmap is None:
            return
            
        # Convert costmap to numpy array
        costmap_data = np.array(self.costmap.data).reshape(self.costmap.info.height, self.costmap.info.width)
        
        # Different resolution and origin? We need to align the maps
        # This is simplified - you may need more complex transformation if maps have different origins/resolutions
        if (self.costmap.info.resolution != self.static_map.info.resolution or
            self.costmap.info.origin.position.x != self.static_map.info.origin.position.x or
            self.costmap.info.origin.position.y != self.static_map.info.origin.position.y):
            rospy.logwarn("Maps have different resolutions or origins - will attempt basic alignment")
        
        # Find occupied cells in costmap that are free in static map
        # 0 in static map means free space
        # >0 in costmap means occupied or inflation
        # Let's consider cells with costmap value > 65 as potential obstacles (you may need to adjust this threshold)
        obstacle_threshold = 65  # Adjust based on your costmap values
        
        # Debug counts
        static_occupied_count = 0
        costmap_occupied_count = 0
        candidate_count = 0
        
        # Lists to store candidate points
        candidate_x = []
        candidate_y = []
        
        # Iterate through the costmap
        for y in range(costmap_data.shape[0]):
            for x in range(costmap_data.shape[1]):
                # Check if this point is within static map bounds
                static_x = int((x * self.costmap.info.resolution + self.costmap.info.origin.position.x - 
                               self.static_map.info.origin.position.x) / self.static_map.info.resolution)
                static_y = int((y * self.costmap.info.resolution + self.costmap.info.origin.position.y - 
                               self.static_map.info.origin.position.y) / self.static_map.info.resolution)
                
                # Get costmap value
                costmap_value = costmap_data[y, x]
                if costmap_value > obstacle_threshold:
                    costmap_occupied_count += 1
                
                if (0 <= static_x < self.static_map.info.width and 
                    0 <= static_y < self.static_map.info.height):
                    
                    # Get values from both maps
                    static_value = self.static_map_data[static_y, static_x]
                    
                    # Count occupied cells in static map
                    if static_value > 0:
                        static_occupied_count += 1
                    
                    # If free in static map but obstacle in costmap
                    if static_value == 0 and costmap_value > obstacle_threshold:
                        # Convert to world coordinates
                        world_x = x * self.costmap.info.resolution + self.costmap.info.origin.position.x
                        world_y = y * self.costmap.info.resolution + self.costmap.info.origin.position.y
                        candidate_x.append(world_x)
                        candidate_y.append(world_y)
                        candidate_count += 1
        
        # Diagnostic info
        rospy.loginfo(f"Map comparison: Found {candidate_count} candidate points")
        rospy.loginfo(f"Static map occupied cells: {static_occupied_count}, Costmap occupied cells: {costmap_occupied_count}")
        
        # Visualize candidate points
        self.visualize_candidate_points(candidate_x, candidate_y)
        
        # Now cluster these points to find distinct workstations
        self.cluster_workstations(candidate_x, candidate_y)
    
    def visualize_candidate_points(self, x_points, y_points):
        """Visualize candidate points as red spheres"""
        marker_array = MarkerArray()
        
        # Limit to 100 markers to avoid overwhelming visualization
        display_count = min(len(x_points), 100)
        
        for i in range(display_count):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "candidate_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x_points[i]
            marker.pose.position.y = y_points[i]
            marker.pose.position.z = 0.1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime = rospy.Duration(5)
            
            marker_array.markers.append(marker)
        
        self.debug_marker_pub.publish(marker_array)
    
    def cluster_workstations(self, x_points, y_points):
        if len(x_points) == 0:
            return
            
        rospy.loginfo(f"Found {len(x_points)} candidate points, clustering...")
        
        # Simple clustering by distance
        clusters = []
        min_distance = 0.3  # Minimum distance between clusters (meters)
        
        for i in range(len(x_points)):
            x, y = x_points[i], y_points[i]
            
            # Check if this point belongs to an existing cluster
            found_cluster = False
            for cluster in clusters:
                # Calculate distance to cluster center
                cx = sum(cluster[0]) / len(cluster[0])
                cy = sum(cluster[1]) / len(cluster[1])
                distance = np.sqrt((x - cx)**2 + (y - cy)**2)
                
                if distance < min_distance:
                    # Add to existing cluster
                    cluster[0].append(x)
                    cluster[1].append(y)
                    found_cluster = True
                    break
            
            if not found_cluster:
                # Create new cluster
                clusters.append([[x], [y]])
        
        # Log the number of clusters and their sizes
        cluster_sizes = [len(c[0]) for c in clusters]
        rospy.loginfo(f"Found {len(clusters)} clusters with sizes: {cluster_sizes}")
        
        # For each cluster, check if it matches our machine dimensions
        self.workstations_detected = []
        
        for cluster in clusters:
            # Reduced minimum cluster size from 5 to 3
            if len(cluster[0]) < 3:
                continue  # Skip too small clusters
                
            # Get the extent of the cluster
            min_x, max_x = min(cluster[0]), max(cluster[0])
            min_y, max_y = min(cluster[1]), max(cluster[1])
            
            width = max_x - min_x
            height = max_y - min_y
            
            # Log the dimensions of each significant cluster
            rospy.loginfo(f"Cluster dimensions: {width:.2f}x{height:.2f}m with {len(cluster[0])} points")
            
            # Check if dimensions approximately match our machine size (with some tolerance)
            # Either orientation could match (length×width or width×length)
            tolerance = 0.3  # 30% tolerance, increased from 20%
            
            orientation = 0  # Default orientation
            is_match = False
            
            # Check if the object matches our expected dimensions in either orientation
            if (abs(width - self.machine_length) < self.machine_length * tolerance and 
                abs(height - self.machine_width) < self.machine_width * tolerance):
                # Machine is horizontal (length along x-axis)
                orientation = 0
                is_match = True
            elif (abs(width - self.machine_width) < self.machine_width * tolerance and 
                  abs(height - self.machine_length) < self.machine_length * tolerance):
                # Machine is vertical (length along y-axis)
                orientation = math.pi / 2
                is_match = True
            
            # If partial view (at least half the expected size), also consider it a match
            if not is_match:
                # Check if it could be a partial view (at least half of original dimensions)
                if (width > self.machine_width * 1.2 and width < self.machine_length and
                    height > self.machine_width * 0.8):
                    # Partial view, horizontal orientation
                    orientation = 0
                    is_match = True
                    rospy.loginfo("Detected partial view of machine (horizontal)")
                elif (height > self.machine_width * 1.2 and height < self.machine_length and
                      width > self.machine_width * 0.8):
                    # Partial view, vertical orientation
                    orientation = math.pi / 2
                    is_match = True
                    rospy.loginfo("Detected partial view of machine (vertical)")
            
            if is_match:
                # It's a match - calculate center
                cx = sum(cluster[0]) / len(cluster[0])
                cy = sum(cluster[1]) / len(cluster[1])
                
                # Create a pose for this workstation
                pose = Pose()
                pose.position.x = cx
                pose.position.y = cy
                pose.position.z = 0.0
                
                # Set orientation using quaternion
                q = tf.transformations.quaternion_from_euler(0, 0, orientation)
                pose.orientation.x = q[0]
                pose.orientation.y = q[1]
                pose.orientation.z = q[2]
                pose.orientation.w = q[3]
                
                self.workstations_detected.append(pose)
                rospy.loginfo(f"Detected machine at ({cx:.2f}, {cy:.2f}) with dimensions {width:.2f}x{height:.2f}m")
        
        self.workstations_count = len(self.workstations_detected)
        rospy.loginfo(f"Found {self.workstations_count} machines matching target dimensions")
        
        # Publish workstations
        self.publish_workstations()
    
    def publish_workstations(self):
        # Publish PoseArray
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.poses = self.workstations_detected
        self.workstation_pub.publish(msg)
        
        # Publish markers for visualization
        marker_array = MarkerArray()
        
        for i, workstation in enumerate(self.workstations_detected):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "workstations"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = workstation
            
            # Set size to match our machine dimensions
            marker.scale.x = self.machine_length
            marker.scale.y = self.machine_width
            marker.scale.z = 0.5  # Height
            
            # Set color (green)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker.lifetime = rospy.Duration(0)  # Persistent
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        detector = WorkstationDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
