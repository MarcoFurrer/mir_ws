#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import tf

class WorkstationDetector:
    def __init__(self):
        rospy.init_node('workstation_detector', anonymous=True)
        
        # Store workstation locations and count
        self.workstations_detected = []
        self.workstations_count = 0
        
        # Publishers
        self.workstation_pub = rospy.Publisher('/detected_workstations', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/workstation_markers', MarkerArray, queue_size=10)
        
        # Subscribers
        self.static_map_sub = rospy.Subscriber('/map', OccupancyGrid, self.static_map_callback)
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        
        # Store maps
        self.static_map = None
        self.static_map_data = None
        self.costmap = None
        
        rospy.loginfo("Enhanced workstation detector initialized")
    
    def static_map_callback(self, map_msg):
        if self.static_map is None:
            self.static_map = map_msg
            self.static_map_data = np.array(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
            rospy.loginfo("Static map received")
    
    def costmap_callback(self, costmap_msg):
        if self.static_map is None:
            return
        
        self.costmap = costmap_msg
        rospy.loginfo("Processing costmap to find workstations")
        
        # Process the costmap to find workstations
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
                
                if (0 <= static_x < self.static_map.info.width and 
                    0 <= static_y < self.static_map.info.height):
                    
                    # Get values from both maps
                    static_value = self.static_map_data[static_y, static_x]
                    costmap_value = costmap_data[y, x]
                    
                    # If free in static map but obstacle in costmap
                    if static_value == 0 and costmap_value > obstacle_threshold:
                        # Convert to world coordinates
                        world_x = x * self.costmap.info.resolution + self.costmap.info.origin.position.x
                        world_y = y * self.costmap.info.resolution + self.costmap.info.origin.position.y
                        candidate_x.append(world_x)
                        candidate_y.append(world_y)
        
        # Now cluster these points to find distinct workstations
        self.cluster_workstations(candidate_x, candidate_y)
    
    def cluster_workstations(self, x_points, y_points):
        if len(x_points) == 0:
            return
            
        rospy.loginfo(f"Found {len(x_points)} candidate points, clustering...")
        
        # Simple clustering by distance
        clusters = []
        min_distance = 0.5  # Minimum distance between clusters (meters)
        
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
        
        # Filter clusters by size - we want substantial objects
        min_points = 5  # Minimum number of points to be considered a workstation
        valid_clusters = [c for c in clusters if len(c[0]) >= min_points]
        
        # Save each cluster as a workstation
        self.workstations_detected = []
        for cluster in valid_clusters:
            # Calculate the center of the cluster
            cx = sum(cluster[0]) / len(cluster[0])
            cy = sum(cluster[1]) / len(cluster[1])
            
            # Create a pose for this workstation
            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = 0.0
            pose.orientation.w = 1.0  # Default orientation
            
            self.workstations_detected.append(pose)
        
        # Limit to 4 workstations (the largest ones)
        if len(self.workstations_detected) > 4:
            # TODO: Use a better metric than just taking the first 4
            self.workstations_detected = self.workstations_detected[:4]
        
        self.workstations_count = len(self.workstations_detected)
        rospy.loginfo(f"Found {self.workstations_count} workstations")
        
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
            
            # Set size (adjust as needed for your workstations)
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
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
