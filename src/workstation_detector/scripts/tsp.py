import rospy

class TSP:
    def __init__(self, cities):
        self.cities = cities
        self.num_cities = len(cities)
        self.best_route = None
        self.best_distance = float('inf')
        self.route_indices = None

    def calculate_distance(self, city1, city2):
        return ((city1[0] - city2[0]) ** 2 + (city1[1] - city2[1]) ** 2) ** 0.5

    def total_distance(self, route):
        distance = 0
        for i in range(len(route)):
            distance += self.calculate_distance(route[i], route[(i + 1) % len(route)])
        return distance

    def find_best_route(self):
        """Use Google OR-Tools for efficient TSP solutions, with fallback to greedy approach"""
        try:
            from ortools.constraint_solver import routing_enums_pb2
            from ortools.constraint_solver import pywrapcp
            
            # Check if we have too few cities for OR-Tools to be necessary
            if self.num_cities <= 1:
                self.best_route = tuple(self.cities)
                self.best_distance = 0
                self.route_indices = list(range(self.num_cities))
                return
                
            rospy.loginfo(f"Using OR-Tools to solve TSP with {self.num_cities} cities")
            
            # Create distance matrix
            dist_matrix = []
            for i in range(self.num_cities):
                row = []
                for j in range(self.num_cities):
                    if i == j:
                        row.append(0)
                    else:
                        row.append(int(self.calculate_distance(self.cities[i], self.cities[j]) * 1000))
                dist_matrix.append(row)
                
            # Create routing model
            manager = pywrapcp.RoutingIndexManager(self.num_cities, 1, 0)  # 1 vehicle, start at node 0
            routing = pywrapcp.RoutingModel(manager)
            
            def distance_callback(from_index, to_index):
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return dist_matrix[from_node][to_node]
                
            transit_callback_index = routing.RegisterTransitCallback(distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
            
            # Set search parameters
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
                
            # Solve the problem
            solution = routing.SolveWithParameters(search_parameters)
            
            if solution:
                route = []
                route_indices = []
                index = routing.Start(0)
                
                while not routing.IsEnd(index):
                    node_index = manager.IndexToNode(index)
                    route.append(self.cities[node_index])
                    route_indices.append(node_index)
                    index = solution.Value(routing.NextVar(index))
                    
                # Add the starting city again to complete the route
                route.append(self.cities[0])
                route_indices.append(0)
                
                self.best_route = tuple(route)
                self.best_distance = self.total_distance(route[:-1])  # Don't double count return to start
                self.route_indices = route_indices
                rospy.loginfo(f"OR-Tools found route with distance: {self.best_distance:.2f}")
                return
        except ImportError:
            rospy.logwarn("Google OR-Tools not installed. Using fallback greedy algorithm.")
            self.find_best_route_greedy()
        except Exception as e:
            rospy.logerr(f"Error using OR-Tools: {e}. Using fallback greedy algorithm.")
            self.find_best_route_greedy()
    
    def find_best_route_greedy(self):
        """Find a good route using the nearest neighbor greedy algorithm"""
        if not self.cities:
            return
        
        rospy.loginfo(f"Using greedy nearest neighbor algorithm for {self.num_cities} cities")
        
        # Start with city 0 (usually the robot position)
        current_city = 0
        unvisited = list(range(1, self.num_cities))  # Skip city 0 (starting point)
        route = [self.cities[current_city]]
        route_indices = [current_city]
        
        # Visit each city in order of closest neighbor
        while unvisited:
            # Find nearest unvisited city
            nearest_idx = -1
            nearest_dist = float('inf')
            current_coords = self.cities[current_city]
            
            for i in unvisited:
                dist = self.calculate_distance(current_coords, self.cities[i])
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_idx = i
                    
            # Move to nearest city
            current_city = nearest_idx
            route.append(self.cities[current_city])
            route_indices.append(current_city)
            unvisited.remove(current_city)
        
        # Complete the route by returning to start
        route.append(self.cities[0])  # Return to start
        route_indices.append(0)
        
        total_dist = self.total_distance(route[:-1])  # Don't double count return to start
        
        self.best_route = tuple(route)
        self.best_distance = total_dist
        self.route_indices = route_indices
        
        rospy.loginfo(f"Greedy algorithm found route with distance: {self.best_distance:.2f}")

    def get_best_route(self):
        return self.best_route, self.best_distance
    
    
if __name__ == "__main__":
    # Example cities with (x, y) coordinates
    example_cities = [
        (0, 0),    # City 1
        (2, 4),    # City 2
        (5, 2),    # City 3
        (8, 7),    # City 4
        (3, 9)     # City 5
    ]
    
    # Initialize the TSP solver with example cities
    tsp_solver = TSP(example_cities)
    
    # Find the best route
    print("Finding the best route...")
    tsp_solver.find_best_route()
    
    # Get and display the result
    best_route, best_distance = tsp_solver.get_best_route()
    
    print("Best Route Found:")
    for i, city in enumerate(best_route):
        print(f"City {i+1}: {city}")
    
    print(f"Total Distance: {best_distance:.2f} units")