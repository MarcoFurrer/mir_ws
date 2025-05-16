class TSP:
    def __init__(self, cities):
        self.cities = cities
        self.num_cities = len(cities)
        self.best_route = None
        self.best_distance = float('inf')

    def calculate_distance(self, city1, city2):
        return ((city1[0] - city2[0]) ** 2 + (city1[1] - city2[1]) ** 2) ** 0.5

    def total_distance(self, route):
        distance = 0
        for i in range(len(route)):
            distance += self.calculate_distance(route[i], route[(i + 1) % len(route)])
        return distance

    def find_best_route(self):
        from itertools import permutations
        for perm in permutations(self.cities):
            current_distance = self.total_distance(perm)
            if current_distance < self.best_distance:
                self.best_distance = current_distance
                self.best_route = perm

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