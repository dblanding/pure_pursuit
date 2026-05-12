#!/usr/bin/env python3
"""
A* path planner for occupancy grid maps
"""
import numpy as np
import cv2
import json
import heapq
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

@dataclass(order=True)
class Node:
    """A* search node"""
    f: float  # Total cost (g + h)
    g: float = field(compare=False)  # Cost from start
    pos: Tuple[int, int] = field(compare=False)  # (row, col)
    parent: Optional['Node'] = field(compare=False, default=None)

class PathPlanner:
    def __init__(self, map_file='map_planning.png', metadata_file='map_metadata.json'):
        """
        Initialize path planner
        
        Args:
            map_file: Binary planning map (inflated obstacles)
            metadata_file: Map metadata JSON
        """
        # Load map
        self.map = cv2.imread(map_file, cv2.IMREAD_GRAYSCALE)
        if self.map is None:
            raise FileNotFoundError(f"Could not load {map_file}")
        
        # Load metadata
        with open(metadata_file, 'r') as f:
            self.metadata = json.load(f)
        
        self.resolution = self.metadata['resolution']
        self.origin_x = self.metadata['origin_x']
        self.origin_y = self.metadata['origin_y']
        
        # Create binary grid (True = free, False = occupied)
        self.grid = self.map > 127
        
        print(f"✅ Loaded map: {self.map.shape}")
        print(f"   Free cells: {np.sum(self.grid)} ({np.sum(self.grid)/self.grid.size*100:.1f}%)")
        print(f"   Resolution: {self.resolution}m/cell")
        print(f"   Origin: ({self.origin_x}, {self.origin_y})")
    
    def world_to_grid(self, x, y):
        """Convert world coordinates (meters) to grid coordinates (cells)"""
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col
    
    def grid_to_world(self, row, col):
        """Convert grid coordinates (cells) to world coordinates (meters)"""
        x = col * self.resolution + self.origin_x
        y = row * self.resolution + self.origin_y
        return x, y
    
    def is_valid(self, row, col):
        """Check if grid cell is valid and free"""
        if row < 0 or row >= self.grid.shape[0]:
            return False
        if col < 0 or col >= self.grid.shape[1]:
            return False
        return self.grid[row, col]
    
    def get_neighbors(self, row, col):
        """
        Get valid 8-connected neighbors
        Returns: list of (row, col, cost) tuples
        """
        neighbors = []
        
        # 8-connected movement (including diagonals)
        moves = [
            (-1, 0, 1.0),   # North
            (1, 0, 1.0),    # South
            (0, -1, 1.0),   # West
            (0, 1, 1.0),    # East
            (-1, -1, 1.414),  # Northwest
            (-1, 1, 1.414),   # Northeast
            (1, -1, 1.414),   # Southwest
            (1, 1, 1.414),    # Southeast
        ]
        
        for dr, dc, cost in moves:
            new_row, new_col = row + dr, col + dc
            if self.is_valid(new_row, new_col):
                neighbors.append((new_row, new_col, cost))
        
        return neighbors
    
    def heuristic(self, row1, col1, row2, col2):
        """Euclidean distance heuristic"""
        return np.sqrt((row1 - row2)**2 + (col1 - col2)**2)
    
    def plan(self, start_x, start_y, goal_x, goal_y):
        """
        A* path planning
        
        Args:
            start_x, start_y: Start position in world coordinates (meters)
            goal_x, goal_y: Goal position in world coordinates (meters)
        
        Returns:
            List of (x, y) waypoints in world coordinates, or None if no path found
        """
        # Convert to grid coordinates
        start_row, start_col = self.world_to_grid(start_x, start_y)
        goal_row, goal_col = self.world_to_grid(goal_x, goal_y)
        
        print(f"\n🎯 Planning path:")
        print(f"   Start: ({start_x:.2f}, {start_y:.2f}) -> grid ({start_row}, {start_col})")
        print(f"   Goal:  ({goal_x:.2f}, {goal_y:.2f}) -> grid ({goal_row}, {goal_col})")
        
        # Validate start and goal
        if not self.is_valid(start_row, start_col):
            print(f"❌ Start position is not valid/free!")
            return None
        if not self.is_valid(goal_row, goal_col):
            print(f"❌ Goal position is not valid/free!")
            return None
        
        # A* search
        open_set = []
        closed_set = set()
        
        # Create start node
        start_node = Node(
            f=0,
            g=0,
            pos=(start_row, start_col),
            parent=None
        )
        heapq.heappush(open_set, start_node)
        
        # Track best g-score for each position
        g_scores = {(start_row, start_col): 0}
        
        nodes_explored = 0
        
        while open_set:
            # Get node with lowest f-score
            current = heapq.heappop(open_set)
            current_pos = current.pos
            
            # Skip if already processed
            if current_pos in closed_set:
                continue
            
            closed_set.add(current_pos)
            nodes_explored += 1
            
            # Check if reached goal
            if current_pos == (goal_row, goal_col):
                print(f"✅ Path found! Explored {nodes_explored} nodes")

                # Reconstruct the raw path
                raw_path = self._reconstruct_path(current)
                print(f"   Raw path: {len(raw_path)} waypoints")
        
                # Smooth the path
                smoothed_path = self.smooth_path(raw_path)
                print(f"   Smoothed: {len(smoothed_path)} waypoints")
        
                return smoothed_path  # Return smoothed version
            
            # Explore neighbors
            for neighbor_row, neighbor_col, move_cost in self.get_neighbors(*current_pos):
                neighbor_pos = (neighbor_row, neighbor_col)
                
                if neighbor_pos in closed_set:
                    continue
                
                # Calculate tentative g-score
                tentative_g = current.g + move_cost
                
                # Skip if not better than existing path
                if neighbor_pos in g_scores and tentative_g >= g_scores[neighbor_pos]:
                    continue
                
                # This is the best path so far
                g_scores[neighbor_pos] = tentative_g
                
                # Calculate heuristic
                h = self.heuristic(neighbor_row, neighbor_col, goal_row, goal_col)
                
                # Create neighbor node
                neighbor_node = Node(
                    f=tentative_g + h,
                    g=tentative_g,
                    pos=neighbor_pos,
                    parent=current
                )
                heapq.heappush(open_set, neighbor_node)
        
        print(f"❌ No path found! Explored {nodes_explored} nodes")
        return None
    
    def _reconstruct_path(self, goal_node):
        """Reconstruct path from goal node to start"""
        path_grid = []
        current = goal_node
        
        while current is not None:
            path_grid.append(current.pos)
            current = current.parent
        
        path_grid.reverse()
        
        # Convert to world coordinates
        path_world = []
        for row, col in path_grid:
            x, y = self.grid_to_world(row, col)
            path_world.append((x, y))
        
        print(f"   Path length: {len(path_world)} waypoints, {self._path_distance(path_world):.2f}m")
        
        return path_world
    
    def _path_distance(self, path):
        """Calculate total path distance"""
        if len(path) < 2:
            return 0.0
        
        distance = 0.0
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            distance += np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        return distance
    
    def visualize_path(self, path, output_file='path_visualization.png'):
        """
        Visualize path on map
        
        Args:
            path: List of (x, y) waypoints
            output_file: Output image file
        """
        # Create color image
        vis = cv2.cvtColor(self.map, cv2.COLOR_GRAY2BGR)
        
        if path is None or len(path) == 0:
            cv2.imwrite(output_file, vis)
            return
        
        # Draw path
        for i in range(len(path) - 1):
            x1, y1 = path[i]
            x2, y2 = path[i + 1]
            
            row1, col1 = self.world_to_grid(x1, y1)
            row2, col2 = self.world_to_grid(x2, y2)
            
            cv2.line(vis, (col1, row1), (col2, row2), (0, 255, 0), 2)
        
        # Draw start (green circle)
        x_start, y_start = path[0]
        row_start, col_start = self.world_to_grid(x_start, y_start)
        cv2.circle(vis, (col_start, row_start), 5, (0, 255, 0), -1)
        
        # Draw goal (red circle)
        x_goal, y_goal = path[-1]
        row_goal, col_goal = self.world_to_grid(x_goal, y_goal)
        cv2.circle(vis, (col_goal, row_goal), 5, (0, 0, 255), -1)
        
        cv2.imwrite(output_file, vis)
        print(f"✅ Saved visualization to {output_file}")

    def smooth_path(self, path, max_iterations=100):
        """
        Smooth path by removing unnecessary waypoints (shortcut method)
        
        Tries to connect non-adjacent waypoints with straight lines,
        removing intermediate points if the line is collision-free.
        
        Args:
            path: List of (x, y) waypoints
            max_iterations: How many smoothing passes to make
        
        Returns:
            Smoothed path with fewer waypoints
        """
        if len(path) <= 2:
            return path
        
        smoothed = list(path)
        
        for iteration in range(max_iterations):
            if len(smoothed) <= 2:
                break
            
            improved = False
            i = 0
            
            while i < len(smoothed) - 2:
                # Try to connect point i directly to point i+2 (skip i+1)
                start = smoothed[i]
                end = smoothed[i + 2]
                
                # Check if direct line is collision-free
                if self._line_is_clear(start, end):
                    # Remove the middle point
                    smoothed.pop(i + 1)
                    improved = True
                else:
                    i += 1
            
            if not improved:
                break
        
        print(f"   Reduced from {len(path)} to {len(smoothed)} waypoints")
        return smoothed

    def _line_is_clear(self, start, end, num_checks=20):
        """
        Check if straight line between two points is collision-free
        
        Args:
            start: (x, y) starting point
            end: (x, y) ending point
            num_checks: Number of points to check along the line
        
        Returns:
            True if line is clear, False if it hits an obstacle
        """
        x1, y1 = start
        x2, y2 = end
        
        for i in range(num_checks + 1):
            t = i / num_checks
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            row, col = self.world_to_grid(x, y)
            if not self.is_valid(row, col):
                return False
        
        return True

def main():
    """Test path planner"""
    planner = PathPlanner()
    
    # Example: Plan path from one room to another
    # Adjust these coordinates based on your map!
    start_x, start_y = 0.0, 0.0    # Start position
    goal_x, goal_y = 5.0, 5.0      # Goal position
    
    path = planner.plan(start_x, start_y, goal_x, goal_y)
    
    if path:
        print(f"\n📍 Waypoints:")
        for i, (x, y) in enumerate(path[::10]):  # Print every 10th waypoint
            print(f"   {i*10}: ({x:.2f}, {y:.2f})")
        
        planner.visualize_path(path)
    else:
        print("❌ Path planning failed!")


if __name__ == '__main__':
    main()
