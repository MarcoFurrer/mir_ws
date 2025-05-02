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
        self.machine_length = 0.70  # 70cm
        self.machine_width = 0.35   # 35cm
        
        # Publishers
        self.workstation_pub = rospy.Publisher('/detected_workstations', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/workstation_markers', MarkerArray, queue_size=10)
        self.debug_marker_pub = rospy.Publisher('/candidate_points', MarkerArray, queue_size=10)
        self.line_marker_pub = rospy.Publisher('/line_candidates', MarkerArray, queue_size=10)
        
        # Subscribers - fixed topic name to match the actual topic
        self.costmap_sub = rospy.Subscriber('/move_base_node/global_costmap/costmap', OccupancyGrid, self.costmap_callback)
        rospy.loginfo("Subscribing to costmap topic: /move_base_node/global_costmap/costmap")
        
        # Store map
        self.costmap = None
        
        # Debug counters
        self.map_updates = 0
        
        # Store detected workstations for persistence
        self.all_detections = []  # Will store all valid workstation detections
        
        # Debug flags
        self.debug_level = 2  # 0=minimal, 1=normal, 2=verbose
        
        rospy.loginfo("Machine detector initialized - looking for objects 70cm x 35cm directly in costmap")
        rospy.loginfo("Debug level set to %d (0=minimal, 1=normal, 2=verbose)", self.debug_level)
    
    def debug_print(self, level, message):
        """Print debug message if debug_level >= level"""
        if self.debug_level >= level:
            rospy.loginfo(message)
    
    def costmap_callback(self, costmap_msg):
        self.map_updates += 1
        self.costmap = costmap_msg
        
        self.debug_print(1, f"Received costmap update #{self.map_updates}, resolution: {costmap_msg.info.resolution}")
        self.debug_print(2, f"Costmap dimensions: {costmap_msg.info.width}x{costmap_msg.info.height}")
        
        # Only process every 5th update to reduce computational load
        if self.map_updates % 5 == 0:
            self.debug_print(1, "Processing costmap to find workstations")
            self.find_workstations()
    
    def find_workstations(self):
        if self.costmap is None:
            return
            
        # Convert costmap to numpy array
        costmap_data = np.array(self.costmap.data).reshape(self.costmap.info.height, self.costmap.info.width)
        
        # Consider cells with costmap value > threshold as potential obstacles
        # Lower threshold to 50 to catch more potential obstacles
        obstacle_threshold = 50  # Adjust based on your costmap values
        
        # Lists to store candidate points (occupied cells)
        candidate_x = []
        candidate_y = []
        
        # Count different types of cells
        unknown_cells = 0
        free_cells = 0
        occupied_cells = 0
        
        # Iterate through the costmap
        for y in range(costmap_data.shape[0]):
            for x in range(costmap_data.shape[1]):                
                # Get costmap value
                costmap_value = costmap_data[y, x]
                
                # Count cell types
                if costmap_value == -1:
                    unknown_cells += 1
                elif costmap_value < obstacle_threshold:
                    free_cells += 1
                else:
                    occupied_cells += 1
                    
                    # Convert to world coordinates
                    world_x = x * self.costmap.info.resolution + self.costmap.info.origin.position.x
                    world_y = y * self.costmap.info.resolution + self.costmap.info.origin.position.y
                    candidate_x.append(world_x)
                    candidate_y.append(world_y)
        
        # Diagnostic info
        self.debug_print(1, f"Costmap analysis: {occupied_cells} occupied, {free_cells} free, {unknown_cells} unknown cells")
        self.debug_print(1, f"Found {len(candidate_x)} candidate points for clustering")
        
        # Display min/max values in costmap for debugging threshold
        if self.debug_level >= 2 and len(costmap_data) > 0:
            costmap_values = costmap_data.flatten()
            costmap_values = costmap_values[costmap_values >= 0]  # Remove unknown (-1) values
            if len(costmap_values) > 0:
                self.debug_print(2, f"Costmap value range: min={np.min(costmap_values)}, max={np.max(costmap_values)}, threshold={obstacle_threshold}")
            
        # Visualize candidate points
        self.visualize_candidate_points(candidate_x, candidate_y)
        
        if len(candidate_x) < 3:
            self.debug_print(1, "Too few candidate points for clustering")
            return
            
        # Use a simple clustering algorithm (no sklearn dependency)
        self.debug_print(1, "Starting custom clustering algorithm")
        clusters = self.simple_clustering(candidate_x, candidate_y)
        self.debug_print(1, f"Found {len(clusters)} clusters of points")
        
        # Detect objects in clusters
        self.analyze_clusters(clusters)
    
    def simple_clustering(self, x_points, y_points, max_distance=0.15):
        """Simple clustering algorithm without using sklearn"""
        if len(x_points) == 0:
            return []
            
        # Convert to list of points
        points = list(zip(x_points, y_points))
        clusters = []
        
        # Sort points by x coordinate for faster neighbor finding
        points.sort(key=lambda p: p[0])
        
        # Keep track of assigned points
        assigned = set()
        
        self.debug_print(2, f"Starting clustering with {len(points)} points, max_distance={max_distance}")
        
        # Process each point
        for i, point in enumerate(points):
            if i in assigned:
                continue
                
            # Start a new cluster
            cluster_points = [point]
            assigned.add(i)
            
            # Find all points within max_distance of any point in the cluster
            queue = [i]
            while queue:
                idx = queue.pop(0)
                px, py = points[idx]
                
                # Check nearby points (optimization: only check nearby points in sorted list)
                # Find approximate bounds to search
                min_x_idx = i
                while min_x_idx > 0 and px - points[min_x_idx-1][0] <= max_distance:
                    min_x_idx -= 1
                    
                max_x_idx = i
                while max_x_idx < len(points) - 1 and points[max_x_idx+1][0] - px <= max_distance:
                    max_x_idx += 1
                    
                # Check points in this range
                for j in range(min_x_idx, max_x_idx + 1):
                    if j not in assigned:
                        qx, qy = points[j]
                        # Calculate actual distance
                        if np.sqrt((qx - px)**2 + (qy - py)**2) <= max_distance:
                            cluster_points.append(points[j])
                            assigned.add(j)
                            queue.append(j)
            
            # Only consider clusters with at least 3 points
            if len(cluster_points) >= 3:
                # Extract x and y coordinates
                cluster_x = [p[0] for p in cluster_points]
                cluster_y = [p[1] for p in cluster_points]
                clusters.append([cluster_x, cluster_y])
                self.debug_print(2, f"Created cluster #{len(clusters)} with {len(cluster_points)} points")
                
        return clusters
    
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
    
    def is_line_like(self, points_x, points_y, threshold=0.9):
        """Check if points form a line-like structure"""
        if len(points_x) < 3:
            return False, 0, 0
            
        try:
            # Calculate principal components
            points = np.column_stack([points_x, points_y])
            mean = np.mean(points, axis=0)
            points_centered = points - mean
            
            # Calculate covariance matrix
            cov = np.cov(points_centered.T)
            
            # Calculate eigenvalues and eigenvectors
            eigenvalues, eigenvectors = np.linalg.eig(cov)
            
            # Sort by eigenvalue
            idx = eigenvalues.argsort()[::-1]
            eigenvalues = eigenvalues[idx]
            eigenvectors = eigenvectors[:, idx]
            
            # If first eigenvalue is much larger than second, points are line-like
            if len(eigenvalues) < 2 or eigenvalues[0] < 0.001:
                return False, 0, 0
                
            ratio = eigenvalues[0] / (eigenvalues[1] + 0.00001)  # Avoid division by zero
            is_line = ratio > threshold
            
            # Principal direction (orientation of the line)
            principal_direction = eigenvectors[:, 0]
            angle = math.atan2(principal_direction[1], principal_direction[0])
            
            # Length of the line approximated by range of points along principal direction
            projected = np.dot(points_centered, principal_direction)
            line_length = max(projected) - min(projected)
            
            self.debug_print(2, f"Line analysis: ratio={ratio:.2f}, angle={angle:.2f}, length={line_length:.2f}m")
            
            return is_line, angle, line_length
        except Exception as e:
            self.debug_print(1, f"Error in is_line_like: {str(e)}")
            return False, 0, 0
    
    def is_rectangle_like(self, points_x, points_y):
        """Check if points form a rectangle-like structure"""
        if len(points_x) < 6:  # Need enough points to form a rectangle
            return False, 0, 0, 0
            
        try:
            # Calculate basic dimensions
            min_x, max_x = min(points_x), max(points_x)
            min_y, max_y = min(points_y), max(points_y)
            width = max_x - min_x
            height = max_y - min_y
            
            # Calculate centroid
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            
            # Check if points are distributed around the perimeter (for rectangle detection)
            # Count points near each edge
            points = np.column_stack([points_x, points_y])
            margin = 0.05  # 5cm margin from edge
            
            top_edge = 0
            bottom_edge = 0
            left_edge = 0
            right_edge = 0
            interior_points = 0
            
            for p in points:
                x, y = p[0], p[1]
                # Check if point is near an edge
                if abs(y - max_y) < margin:
                    top_edge += 1
                elif abs(y - min_y) < margin:
                    bottom_edge += 1
                elif abs(x - min_x) < margin:
                    left_edge += 1
                elif abs(x - max_x) < margin:
                    right_edge += 1
                else:
                    interior_points += 1
                    
            # If we have points on at least 3 edges and not too many interior points, it's likely a rectangle
            edges_with_points = sum([1 for e in [top_edge, bottom_edge, left_edge, right_edge] if e > 0])
            
            self.debug_print(2, f"Rectangle analysis: size={width:.2f}x{height:.2f}m, edges={edges_with_points}, " + 
                             f"edge points: top={top_edge}, bottom={bottom_edge}, left={left_edge}, right={right_edge}, interior={interior_points}")
            
            # Rectangle criteria: at least 3 edges with points, and interior points < 50% of total
            is_rectangle = edges_with_points >= 3 and interior_points < len(points_x) * 0.5
            
            return is_rectangle, 0, width, height  # Angle is 0 for now (aligned with axes)
        except Exception as e:
            self.debug_print(1, f"Error in is_rectangle_like: {str(e)}")
            return False, 0, 0, 0
    
    def analyze_clusters(self, clusters):
        """Analyze clusters to find workstations"""
        self.workstations_detected = []
        
        # Visualize line-like clusters
        line_markers = MarkerArray()
        marker_id = 0
        
        # Print each cluster's bounding box for debugging
        for i, cluster in enumerate(clusters):
            if len(cluster[0]) < 3:
                continue
                
            # Get the cluster points
            cluster_x, cluster_y = cluster[0], cluster[1]
            
            # Get basic dimensions
            min_x, max_x = min(cluster_x), max(cluster_x)
            min_y, max_y = min(cluster_y), max(cluster_y)
            width = max_x - min_x
            height = max_y - min_y
            
            self.debug_print(1, f"Cluster #{i+1}: {len(cluster_x)} points, size={width:.2f}x{height:.2f}m")
        
        for cluster in clusters:
            # Skip too small clusters
            if len(cluster[0]) < 3:
                continue
                
            # Get the cluster points
            cluster_x, cluster_y = cluster[0], cluster[1]
            
            # Calculate cluster center
            center_x = sum(cluster_x) / len(cluster_x)
            center_y = sum(cluster_y) / len(cluster_y)
            
            # Get basic dimensions
            min_x, max_x = min(cluster_x), max(cluster_x)
            min_y, max_y = min(cluster_y), max(cluster_y)
            width = max_x - min_x
            height = max_y - min_y
            
            # First check if cluster forms a rectangle
            is_rect, rect_angle, rect_width, rect_height = self.is_rectangle_like(cluster_x, cluster_y)
            
            if is_rect:
                self.debug_print(1, f"Rectangle-like cluster detected with dimensions {rect_width:.2f}x{rect_height:.2f}m")
                
                # Check if dimensions match our target (with tolerance)
                tolerance = 0.3  # 30% tolerance
                orientation = 0  # Default orientation
                is_match = False
                
                # Check if matches expected dimensions in either orientation
                if (abs(rect_width - self.machine_length) < self.machine_length * tolerance and 
                    abs(rect_height - self.machine_width) < self.machine_width * tolerance):
                    # Machine is horizontal (length along x-axis)
                    orientation = 0
                    is_match = True
                elif (abs(rect_width - self.machine_width) < self.machine_width * tolerance and 
                      abs(rect_height - self.machine_length) < self.machine_length * tolerance):
                    # Machine is vertical (length along y-axis)
                    orientation = math.pi / 2
                    is_match = True
                
                if is_match:
                    # Create a pose for this workstation
                    pose = Pose()
                    pose.position.x = center_x
                    pose.position.y = center_y
                    pose.position.z = 0.0
                    
                    # Set orientation using quaternion
                    q = tf.transformations.quaternion_from_euler(0, 0, orientation)
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    
                    self.workstations_detected.append(pose)
                    self.debug_print(1, f"Detected rectangular machine at ({center_x:.2f}, {center_y:.2f}) with dimensions {rect_width:.2f}x{rect_height:.2f}m")
                    continue  # Skip line analysis for this cluster
            
            # If not a rectangle, check if it's a line
            is_line, angle, line_length = self.is_line_like(cluster_x, cluster_y)
            
            # Debug info
            if is_line:
                self.debug_print(1, f"Line-like cluster detected with length {line_length:.2f}m and angle {angle:.2f}")
                
                # Visualize line as a cylinder
                line_marker = Marker()
                line_marker.header.frame_id = "map"
                line_marker.header.stamp = rospy.Time.now()
                line_marker.ns = "line_clusters"
                line_marker.id = marker_id
                marker_id += 1
                line_marker.type = Marker.CYLINDER
                line_marker.action = Marker.ADD
                
                # Set position at center of the line
                line_marker.pose.position.x = center_x
                line_marker.pose.position.y = center_y
                line_marker.pose.position.z = 0.25
                
                # Set orientation along the line angle
                q = tf.transformations.quaternion_from_euler(0, 0, angle)
                line_marker.pose.orientation.x = q[0]
                line_marker.pose.orientation.y = q[1]
                line_marker.pose.orientation.z = q[2]
                line_marker.pose.orientation.w = q[3]
                
                # Set dimensions - length of the line, small width
                line_marker.scale.x = line_length
                line_marker.scale.y = 0.05  # Thin width
                line_marker.scale.z = 0.5   # Height
                
                # Set color (blue)
                line_marker.color.r = 0.0
                line_marker.color.g = 0.0
                line_marker.color.b = 1.0
                line_marker.color.a = 0.7
                
                line_marker.lifetime = rospy.Duration(5)  # Short-lived
                line_markers.markers.append(line_marker)
                
                # Detect machine from line segment
                if self.machine_length * 0.5 <= line_length <= self.machine_length * 1.2:
                    # This line segment could be one edge of a machine
                    # Estimate full machine dimensions
                    
                    # Create a pose with orientation perpendicular to the line
                    pose = Pose()
                    pose.position.x = center_x
                    pose.position.y = center_y
                    pose.position.z = 0.0
                    
                    # Decide orientation based on line angle
                    machine_angle = angle
                    if line_length > self.machine_width * 1.2:
                        # Line is likely the long edge of machine
                        self.debug_print(1, f"Line appears to be long edge of a machine")
                    else:
                        # Line is likely the short edge, rotate 90 degrees
                        machine_angle += math.pi/2
                        self.debug_print(1, f"Line appears to be short edge of a machine")
                        
                    # Set orientation
                    q = tf.transformations.quaternion_from_euler(0, 0, machine_angle)
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    
                    self.workstations_detected.append(pose)
                    self.debug_print(1, f"Estimated machine at ({center_x:.2f}, {center_y:.2f}) from line detection")
            else:
                # Not a line or rectangle - use regular detection with bounding box
                self.debug_print(2, f"Regular cluster with dimensions {width:.2f}x{height:.2f}m with {len(cluster_x)} points")
                
                # Check for matching dimensions
                tolerance = 0.3  # 30% tolerance
                orientation = 0  # Default orientation
                is_match = False
                
                # Check if matches expected dimensions in either orientation
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
                    # Check if it could be a partial view
                    if ((width > self.machine_width * 1.2 and width < self.machine_length * 1.1 and
                        height > self.machine_width * 0.7)):
                        # Partial view, horizontal orientation
                        orientation = 0
                        is_match = True
                        self.debug_print(1, "Detected partial view of machine (horizontal)")
                    elif ((height > self.machine_width * 1.2 and height < self.machine_length * 1.1 and
                          width > self.machine_width * 0.7)):
                        # Partial view, vertical orientation
                        orientation = math.pi / 2
                        is_match = True
                        self.debug_print(1, "Detected partial view of machine (vertical)")
                
                if is_match:
                    # Create a pose for this workstation
                    pose = Pose()
                    pose.position.x = center_x
                    pose.position.y = center_y
                    pose.position.z = 0.0
                    
                    # Set orientation using quaternion
                    q = tf.transformations.quaternion_from_euler(0, 0, orientation)
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    
                    self.workstations_detected.append(pose)
                    self.debug_print(1, f"Detected machine at ({center_x:.2f}, {center_y:.2f}) with dimensions {width:.2f}x{height:.2f}m")
        
        # Publish line markers
        self.line_marker_pub.publish(line_markers)
        
        # Merge with past detections for stability
        self.merge_with_past_detections()
        
        # Publish final workstation markers
        self.publish_workstations()
    
    def merge_with_past_detections(self, persistence_threshold=5):
        """Merge current detections with past ones for stability"""
        # Remove old detections that haven't been seen in a while
        current_time = rospy.Time.now()
        
        # Filter the all_detections list to remove old entries
        self.all_detections = [
            detection for detection in self.all_detections 
            if (current_time - detection["last_seen"]).to_sec() < persistence_threshold
        ]
        
        # Update existing detections or add new ones
        for pose in self.workstations_detected:
            x, y = pose.position.x, pose.position.y
            merged = False
            
            # Check if this detection matches any existing ones
            for detection in self.all_detections:
                existing_x = detection["pose"].position.x
                existing_y = detection["pose"].position.y
                
                # If close enough, consider it the same workstation
                if math.sqrt((x - existing_x)**2 + (y - existing_y)**2) < 0.5:  # 50cm threshold
                    # Update position with moving average
                    alpha = 0.3  # Weight for new observation
                    detection["pose"].position.x = (1-alpha) * existing_x + alpha * x
                    detection["pose"].position.y = (1-alpha) * existing_y + alpha * y
                    detection["count"] += 1
                    detection["last_seen"] = current_time
                    merged = True
                    break
            
            # If no match found, add as new detection
            if not merged:
                self.all_detections.append({
                    "pose": pose,
                    "count": 1,
                    "last_seen": current_time
                })
        
        # Update final list with stable detections (seen multiple times)
        self.workstations_detected = [
            detection["pose"] for detection in self.all_detections 
            if detection["count"] >= 1  # Changed from 2 to 1 for faster detection
        ]
        
        self.debug_print(1, f"After merging: {len(self.workstations_detected)} stable workstations")
    
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
