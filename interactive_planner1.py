#!/usr/bin/env python3
"""
Interactive path planning - click to set start/goal
revised to save waypoints to planned_path.json
"""
import cv2
import numpy as np
from path_planner import PathPlanner

class InteractivePlanner:
    def __init__(self):
        self.planner = PathPlanner()
        self.start = None
        self.goal = None
        self.current_path = None  # Store the last planned path
        
        # Load planning map for visualization
        self.map_img = cv2.imread('map_planning.png', cv2.IMREAD_GRAYSCALE)
        self.display = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        # Color free space green for clarity
        free_mask = self.map_img > 127
        self.display[free_mask] = [0, 200, 0]  # Green
        
        self.update_display()
    
    def update_display(self):
        """Redraw display with current start/goal"""
        # Reset to base map
        self.display = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        free_mask = self.map_img > 127
        self.display[free_mask] = [0, 200, 0]
        
        # Draw start (blue)
        if self.start:
            row, col = self.start
            cv2.circle(self.display, (col, row), 8, (255, 0, 0), -1)
            cv2.circle(self.display, (col, row), 10, (255, 255, 255), 2)
        
        # Draw goal (red)
        if self.goal:
            row, col = self.goal
            cv2.circle(self.display, (col, row), 8, (0, 0, 255), -1)
            cv2.circle(self.display, (col, row), 10, (255, 255, 255), 2)
        
        cv2.imshow('Path Planner - Click Start (blue) then Goal (red)', self.display)
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks"""
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        
        # Check if clicked position is valid
        if y < 0 or y >= self.map_img.shape[0] or x < 0 or x >= self.map_img.shape[1]:
            print(f"❌ Click outside map bounds")
            return
        
        if self.map_img[y, x] < 127:
            print(f"❌ Clicked on obstacle at ({x}, {y})")
            return
        
        # Convert to world coordinates
        world_x, world_y = self.planner.grid_to_world(y, x)
        
        # Set start or goal
        if self.start is None:
            self.start = (y, x)
            print(f"✅ Start set: grid({y}, {x}) = world({world_x:.2f}, {world_y:.2f})")
        elif self.goal is None:
            self.goal = (y, x)
            print(f"✅ Goal set: grid({y}, {x}) = world({world_x:.2f}, {world_y:.2f})")
            print(f"\n🚀 Planning path...")
            self.plan_path()
        else:
            # Reset and start over
            print(f"\n🔄 Resetting - click new start position")
            self.start = (y, x)
            self.goal = None
            self.current_path = None
            print(f"✅ Start set: grid({y}, {x}) = world({world_x:.2f}, {world_y:.2f})")
        
        self.update_display()
    
    def plan_path(self):
        """Plan and visualize path"""
        if not self.start or not self.goal:
            return
        
        # Convert to world coordinates
        start_x, start_y = self.planner.grid_to_world(self.start[0], self.start[1])
        goal_x, goal_y = self.planner.grid_to_world(self.goal[0], self.goal[1])
        
        # Plan path
        path = self.planner.plan(start_x, start_y, goal_x, goal_y)
        
        if path:
            self.current_path = path  # Store for saving
            
            # Draw path on display
            for i in range(len(path) - 1):
                x1, y1 = path[i]
                x2, y2 = path[i + 1]
                
                row1, col1 = self.planner.world_to_grid(x1, y1)
                row2, col2 = self.planner.world_to_grid(x2, y2)
                
                cv2.line(self.display, (col1, row1), (col2, row2), (255, 255, 0), 2)
            
            # Redraw start/goal on top
            cv2.circle(self.display, (self.start[1], self.start[0]), 8, (255, 0, 0), -1)
            cv2.circle(self.display, (self.goal[1], self.goal[0]), 8, (0, 0, 255), -1)
            
            cv2.imshow('Path Planner - Click Start (blue) then Goal (red)', self.display)
            
            # Save visualization
            cv2.imwrite('path_result.png', self.display)
            print(f"✅ Saved path_result.png")
            
            # Save path JSON for path follower
            self.planner.save_path(path, 'planned_path.json')
            
            print(f"\n💡 Options:")
            print(f"   • Click anywhere to plan another path")
            print(f"   • Press 'S' to save path again")
            print(f"   • Press 'F' to follow this path (runs path_follower.py)")
            print(f"   • Press ESC/Q to quit")
        else:
            self.current_path = None
            print(f"❌ No path found!")
            print(f"💡 Click to try different start/goal")
    
    def save_current_path(self):
        """Save current path to JSON"""
        if self.current_path:
            filename = input("\nEnter filename (default: planned_path.json): ").strip()
            if not filename:
                filename = 'planned_path.json'
            self.planner.save_path(self.current_path, filename)
        else:
            print("❌ No path to save! Plan a path first.")
    
    def follow_current_path(self):
        """Launch path follower with current path"""
        if self.current_path:
            print("\n🚀 Launching path follower...")
            print("   (Close this window or it will continue running)")
            import subprocess
            try:
                subprocess.Popen(['python3', 'path_follower.py', '--path-file', 'planned_path.json'])
                print("✅ Path follower started!")
            except Exception as e:
                print(f"❌ Could not start path follower: {e}")
                print("💡 Run manually: python3 path_follower.py")
        else:
            print("❌ No path to follow! Plan a path first.")
    
    def run(self):
        """Run interactive planner"""
        print("=" * 60)
        print("🗺️  INTERACTIVE PATH PLANNER")
        print("=" * 60)
        print("📍 Click on GREEN area to set START (blue circle)")
        print("🎯 Click on GREEN area to set GOAL (red circle)")
        print("🚀 Path will be calculated automatically")
        print("🔄 Click again to plan a new path")
        print("💾 Press 'S' to save path with custom filename")
        print("🤖 Press 'F' to follow path (launch path_follower.py)")
        print("❌ Press ESC or Q to quit")
        print("=" * 60)
        
        cv2.namedWindow('Path Planner - Click Start (blue) then Goal (red)')
        cv2.setMouseCallback('Path Planner - Click Start (blue) then Goal (red)', self.mouse_callback)
        
        self.update_display()
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):  # ESC or Q
                break
            elif key == ord('s') or key == ord('S'):  # Save
                self.save_current_path()
            elif key == ord('f') or key == ord('F'):  # Follow
                self.follow_current_path()
        
        cv2.destroyAllWindows()
        print("\n👋 Goodbye!")


if __name__ == '__main__':
    planner = InteractivePlanner()
    planner.run()
