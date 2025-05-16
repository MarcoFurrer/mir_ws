#!/usr/bin/env python
import rospy
import actionlib
import random
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseArray, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray
import tf
import math

class ExplorationController:
    def __init__(self):
        rospy.init_node('exploration_controller')
        
        # Track found machines
        self.machines = []
        self.machines_found = 0
        self.machine_visited = []  # Keep track of which machines we've visited
        self.target_machines = 4  # Target number of machines to find
        
        # Store frontier points to explore
        self.frontier_points = []
        self.current_goal = None
        self.exploring = False
        self.current_state = "EXPLORING"  # States: EXPLORING, VISITING_MACHINE, DONE
        
        # Move base client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")
        
        # Publishers and Subscribers
        self.frontier_pub = rospy.Publisher('/exploration_frontiers', MarkerArray, queue_size=10)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.workstations_sub = rospy.Subscriber('/detected_workstations', PoseArray, self.workstations_callback)
        
        rospy.loginfo("Exploration controller initialized")
        self.rate = rospy.Rate(1)  # 1Hz refresh
        
        # Start exploration after a delay
        rospy.Timer(rospy.Duration(3), self.start_exploration, oneshot=True)
        
    def start_exploration(self, event):
        # Start the main control loop
        self.main_loop()
        
    def workstations_callback(self, workstations_msg):
        self.machines = workstations_msg.poses
        new_count = len(workstations_msg.poses)
        
        # If we found new machines, update our records
        if new_count > self.machines_found:
            rospy.loginfo(f"New machines detected! Now at {new_count}/{self.target_machines}")
            # Initialize visited status for any new machines
            while len(self.machine_visited) < new_count:
                self.machine_visited.append(False)
                
        self.machines_found = new_count
        
        # If we found all machines and we're exploring, switch to visiting them
        if self.machines_found >= self.target_machines and self.current_state == "EXPLORING":
            rospy.loginfo("All machines found! Switching to machine visit mode.")
            # Cancel current goal if exploring
            if self.exploring and self.current_goal is not None:
                self.move_base_client.cancel_goal()
                self.exploring = False
            self.current_state = "VISITING_MACHINE"
            
    def map_callback(self, map_data):
        # Calculate exploration frontiers based on the map
        if self.current_state == "EXPLORING" and not self.exploring:
            self.calculate_frontiers(map_data)
    
    def calculate_frontiers(self, map_data):
        """Find frontiers (boundaries between explored and unexplored areas)"""
        width = map_data.info.width
        height = map_data.info.height
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        
        # Clear old frontiers
        self.frontier_points = []
        
        # Simple frontier detection
        for y in range(1, height-1):
            for x in range(1, width-1):
                # Current cell is free space
                if map_data.data[y*width + x] == 0:
                    # Check neighbors for unknown space
                    has_unknown = False
                    for dy in [-1, 0, 1]:
                        for dx in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            if map_data.data[(y+dy)*width + (x+dx)] == -1:
                                has_unknown = True
                                break
                        if has_unknown:
                            break
                    
                    if has_unknown:
                        # This is a frontier point
                        world_x = origin_x + (x * resolution)
                        world_y = origin_y + (y * resolution)
                        
                        # Only add if not too close to existing frontier points
                        too_close = False
                        for fx, fy in self.frontier_points:
                            if ((fx - world_x)**2 + (fy - world_y)**2) < (1.0**2):  # 1m threshold
                                too_close = True
                                break
                                
                        if not too_close:
                            self.frontier_points.append((world_x, world_y))
        
        rospy.loginfo(f"Found {len(self.frontier_points)} frontier points")
        self.publish_frontiers()
        
    def publish_frontiers(self):
        """Visualize frontiers in RViz"""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(self.frontier_points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "frontiers"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(5)
            
            marker_array.markers.append(marker)
            
        self.frontier_pub.publish(marker_array)
    
    def select_next_frontier(self):
        """Choose the next frontier to explore"""
        if not self.frontier_points:
            return None
            
        # For simplicity, just pick a random frontier
        # A better strategy would choose based on distance, information gain, etc.
        return random.choice(self.frontier_points)
    
    def select_next_unvisited_machine(self):
        """Select the next machine that hasn't been visited yet"""
        for i in range(len(self.machines)):
            if not self.machine_visited[i]:
                return i
        return None  # All machines visited
    
    def main_loop(self):
        """Main control loop for exploration and machine visiting"""
        while not rospy.is_shutdown():
            if self.current_state == "EXPLORING":
                # Exploration state - find frontiers and navigate to them
                if not self.exploring:
                    frontier = self.select_next_frontier()
                    
                    if frontier:
                        rospy.loginfo(f"Moving to frontier at {frontier}")
                        self.navigate_to_point(frontier[0], frontier[1])
                        self.exploring = True
                    else:
                        rospy.loginfo("No frontiers found. Waiting for map updates.")
            
            elif self.current_state == "VISITING_MACHINE":
                # Visit each machine in turn
                if not self.exploring:
                    next_machine = self.select_next_unvisited_machine()
                    
                    if next_machine is not None:
                        machine_pose = self.machines[next_machine]
                        rospy.loginfo(f"Moving to machine {next_machine} at position ({machine_pose.position.x}, {machine_pose.position.y})")
                        
                        # Calculate approach position (offset from machine center)
                        # Get orientation as Euler angles
                        orientation = machine_pose.orientation
                        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
                        euler = tf.transformations.euler_from_quaternion(quaternion)
                        yaw = euler[2]
                        
                        # Calculate approach position (1 meter away from machine in the opposite direction of its orientation)
                        approach_distance = 1.0
                        approach_x = machine_pose.position.x - approach_distance * math.cos(yaw)
                        approach_y = machine_pose.position.y - approach_distance * math.sin(yaw)
                        
                        # Navigate to approach position
                        self.navigate_to_point(approach_x, approach_y)
                        self.exploring = True
                        self.machine_visited[next_machine] = True
                    else:
                        rospy.loginfo("All machines have been visited!")
                        self.current_state = "DONE"
            
            elif self.current_state == "DONE":
                rospy.loginfo("Exploration and machine visiting complete.")
                # Maybe add some final behavior here
                break
            
            self.rate.sleep()
    
    def navigate_to_point(self, x, y):
        """Send a goal to move_base to navigate to a point"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        goal.target_pose.pose.orientation.w = 1.0  # No rotation, just position
        
        self.current_goal = (x, y)
        self.move_base_client.send_goal(
            goal,
            done_cb=self.goal_done_callback
        )
    
    def goal_done_callback(self, status, result):
        """Called when a navigation goal completes"""
        self.exploring = False
        self.current_goal = None
        
        # Continue in the next loop iteration
        rospy.loginfo(f"Completed navigation goal, current state: {self.current_state}")

if __name__ == '__main__':
    try:
        controller = ExplorationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
