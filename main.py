import heapq
import math
import time
import random

# -------------------------------------------------------------------------
# 1. Data Structures & Grid Environment
# -------------------------------------------------------------------------

class Node:
    """
    Represents a specific spot on the grid (x, y).
    Holds the costs (g, h, f) and the parent node for path reconstruction.
    """
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to here
        self.h = 0  # Estimated cost from here to end
        self.f = 0  # Total cost (g + h)

    # This function allows the Priority Queue (heapq) to compare Nodes based on 'f' score
    def __lt__(self, other):
        return self.f < other.f

class GridGraph:
    """
    Represents the 2D map. 
    Handles boundaries and random obstacle placement.
    """
    def __init__(self, width, height, obstacle_prob=0.2):
        self.width = width
        self.height = height
        self.walls = set()
        
        # Generate random obstacles based on probability
        # We ensure (0,0) is never a wall later in the harness
        for x in range(width):
            for y in range(height):
                if random.random() < obstacle_prob:
                    self.walls.add((x, y))

    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height

    def passable(self, id):
        return id not in self.walls

    def get_neighbors(self, id):
        (x, y) = id
        # Define movement: Up, Down, Left, Right (No diagonals for this simplified version)
        results = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        
        # Filter out positions that are off the map or are walls
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

# -------------------------------------------------------------------------
# 2. Heuristic Functions
# -------------------------------------------------------------------------

def heuristic_manhattan(a, b):
    """
    Manhattan distance: |x1 - x2| + |y1 - y2|
    Best for grid maps where you can only move in 4 directions.
    """
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def heuristic_euclidean(a, b):
    """
    Euclidean distance: Straight line distance.
    Often underestimates cost on a grid, leading to more node expansion.
    """
    (x1, y1) = a
    (x2, y2) = b
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# -------------------------------------------------------------------------
# 3. The A* Algorithm
# -------------------------------------------------------------------------

def a_star_search(graph, start, end, heuristic_func):
    """
    The core A* algorithm.
    - Uses heapq for an Efficient Binary Heap Priority Queue.
    - Returns: (path, nodes_expanded_count, time_taken)
    """
    nodes_expanded = 0
    start_time = time.time()

    open_list = [] # The Priority Queue
    closed_set = set()
    
    # Initialize start node
    start_node = Node(start)
    start_node.g = 0
    start_node.h = heuristic_func(start, end)
    start_node.f = start_node.g + start_node.h
    
    # Push start node to the heap
    heapq.heappush(open_list, start_node)
    
    # Dictionary to keep track of lowest cost found to reach a node so far
    cost_so_far = {start: 0}

    while open_list:
        # Pop the node with the lowest 'f' score (Optimized O(log n) operation)
        current_node = heapq.heappop(open_list)
        nodes_expanded += 1

        # Check if we reached the goal
        if current_node.position == end:
            end_time = time.time()
            # Reconstruct path by backtracking parents
            path = []
            curr = current_node
            while curr is not None:
                path.append(curr.position)
                curr = curr.parent
            return path[::-1], nodes_expanded, (end_time - start_time)

        closed_set.add(current_node.position)

        # Explore neighbors
        for next_pos in graph.get_neighbors(current_node.position):
            if next_pos in closed_set:
                continue

            # Assume cost of moving from one square to the next is 1
            new_cost = cost_so_far[current_node.position] + 1
            
            # If we found a shorter path to this neighbor, update it
            if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                cost_so_far[next_pos] = new_cost
                neighbor = Node(next_pos, current_node)
                neighbor.g = new_cost
                # Dynamic Heuristic Call here:
                neighbor.h = heuristic_func(next_pos, end)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, neighbor)

    # If queue is empty and goal not reached
    return None, nodes_expanded, (time.time() - start_time)

# -------------------------------------------------------------------------
# 4. Testing Harness & Execution
# -------------------------------------------------------------------------

def run_tests():
    print("Initializing Advanced A* System with Dynamic Heuristic Tuning...")
    print("=" * 70)
    
    # Create a 50x50 grid with 30% obstacles
    grid = GridGraph(50, 50, obstacle_prob=0.3)
    start = (0, 0)
    end = (45, 45)

    # Guarantee start and end are traversable
    if start in grid.walls: grid.walls.remove(start)
    if end in grid.walls: grid.walls.remove(end)

    print(f"Map Size: 50x50 | Start: {start} | End: {end}")
    print("-" * 70)
    print(f"{'Heuristic Function':<20} | {'Path Length':<12} | {'Nodes Expanded':<15} | {'Time (sec)':<10}")
    print("-" * 70)

    # Dynamic Heuristic List for comparison
    heuristics = [
        ("Manhattan", heuristic_manhattan),
        ("Euclidean", heuristic_euclidean)
    ]

    for name, func in heuristics:
        path, nodes, duration = a_star_search(grid, start, end, func)
        
        path_len_str = str(len(path)) if path else "No Path"
        
        print(f"{name:<20} | {path_len_str:<12} | {nodes:<15} | {duration:.6f}")

    print("=" * 70)
    print("Analysis Complete.")

if __name__ == "__main__":
    run_tests()