import heapq
import math
import time
import random


class Node:
    """
    Represents a specific spot on the grid (x, y).
    Holds the costs (g, h, f) and the parent node for path reconstruction.
    """
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  
        self.h = 0  
        self.f = 0  

    
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
        
        results = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
        
        
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results


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


def a_star_search(graph, start, end, heuristic_func):
    """
    The core A* algorithm.
    - Uses heapq for an Efficient Binary Heap Priority Queue.
    - Returns: (path, nodes_expanded_count, time_taken)
    """
    nodes_expanded = 0
    start_time = time.time()

    open_list = [] 
    closed_set = set()
    
    
    start_node = Node(start)
    start_node.g = 0
    start_node.h = heuristic_func(start, end)
    start_node.f = start_node.g + start_node.h
    
    
    heapq.heappush(open_list, start_node)
    
    
    cost_so_far = {start: 0}

    while open_list:
        
        current_node = heapq.heappop(open_list)
        nodes_expanded += 1

        
        if current_node.position == end:
            end_time = time.time()
            
            path = []
            curr = current_node
            while curr is not None:
                path.append(curr.position)
                curr = curr.parent
            return path[::-1], nodes_expanded, (end_time - start_time)

        closed_set.add(current_node.position)

        
        for next_pos in graph.get_neighbors(current_node.position):
            if next_pos in closed_set:
                continue

            
            new_cost = cost_so_far[current_node.position] + 1
            
           
            if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                cost_so_far[next_pos] = new_cost
                neighbor = Node(next_pos, current_node)
                neighbor.g = new_cost
                
                neighbor.h = heuristic_func(next_pos, end)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, neighbor)

    
    return None, nodes_expanded, (time.time() - start_time)



def run_tests():
    print("Initializing Advanced A* System with Dynamic Heuristic Tuning...")
    print("=" * 70)
    
    
    grid = GridGraph(50, 50, obstacle_prob=0.3)
    start = (0, 0)
    end = (45, 45)

    
    if start in grid.walls: grid.walls.remove(start)
    if end in grid.walls: grid.walls.remove(end)

    print(f"Map Size: 50x50 | Start: {start} | End: {end}")
    print("-" * 70)
    print(f"{'Heuristic Function':<20} | {'Path Length':<12} | {'Nodes Expanded':<15} | {'Time (sec)':<10}")
    print("-" * 70)

    
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