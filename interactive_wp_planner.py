# interactive_wp_planner.py

#!/usr/bin/env python3
"""
Interactive path planning - click unlimited waypoints
Press 'F' to mark final waypoint and plan path
"""
import cv2
import numpy as np
from path_planner import PathPlanner

class InteractivePlanner:
    def __init__(self):
        self.planner = PathPlanner()
        self.waypoints = []  # List of (row, col, world_x, world_y)
        self.current_path = None
        self.is_final = False  # Flag to indicate if we're ready to plan
        
        # Load planning map for visualization
        self.map_img = cv2.imread('map_planning.png', cv2.IMREAD_GRAYSCALE)
        self.display = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        # Color free space green for clarity
        free_mask = self.map_img > 127
        self.display[free_mask] = [0, 200, 0]
        
        self.update_display()
    
    def update_display(self):
        """Redraw display with current waypoints"""
        # Reset to base map
        self.display = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        free_mask = self.map_img > 127
        self.display[free_mask] = [0, 200, 0]
        
        # Draw waypoints
        for i, (row, col, wx, wy) in enumerate(self.waypoints):
            if i == 0:
                # Start = blue
                color = (255, 0, 0)
            elif i == len(self.waypoints) - 1 and self.is_final:
                # Final waypoint = red
                color = (0, 0, 255)
            else:
                # Intermediate = yellow
                color = (0, 255, 255)
            
            cv2.circle(self.display, (col, row), 8, color, -1)
            cv2.circle(self.display, (col, row), 10, (255, 255, 255), 2)
            
            # Draw waypoint number
            cv2.putText(self.display, str(i+1), (col+12, row+5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Draw lines connecting waypoints
        if len(self.waypoints) > 1:
            for i in range(len(self.waypoints) - 1):
                row1, col1 = self.waypoints[i][:2]
                row2, col2 = self.waypoints[i+1][:2]
                cv2.line(self.display, (col1, row1), (col2, row2), (128, 128, 128), 2)
        
        cv2.imshow('Path Planner - Click waypoints, press F for final, ESC to quit', self.display)
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks"""
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        
        # If already planned, reset
        if self.current_path is not None:
            print(f"\n🔄 Resetting for new path")
            self.reset()
        
        # Check if clicked position is valid
        if y < 0 or y >= self.map_img.shape[0] or x < 0 or x >= self.map_img.shape[1]:
            print(f"❌ Click outside map bounds")
            return
        
        if self.map_img[y, x] < 127:
            print(f"❌ Clicked on obstacle at ({x}, {y})")
            return
        
        # Convert to world coordinates
        world_x, world_y = self.planner.grid_to_world(y, x)
        
        # Add waypoint
        self.waypoints.append((y, x, world_x, world_y))
        self.is_final = False  # New waypoint added, not final yet
        
        print(f"✅ Waypoint {len(self.waypoints)}: grid({y}, {x}) = world({world_x:.2f}, {world_y:.2f})")
        
        if len(self.waypoints) == 1:
            print(f"   💡 Keep clicking to add more waypoints")
            print(f"   💡 Press 'F' when you click the FINAL waypoint")
        
        self.update_display()
    
    def mark_final_and_plan(self):
        """Mark last waypoint as final and plan path"""
        if len(self.waypoints) < 2:
            print(f"❌ Need at least 2 waypoints! (have {len(self.waypoints)})")
            print(f"   💡 Click more waypoints first")
            return
        
        self.is_final = True
        self.update_display()  # Redraw with final waypoint in red
        
        print(f"\n🎯 Final waypoint marked!")
        print(f"🚀 Planning path through {len(self.waypoints)} waypoints...")
        
        # Plan segments between consecutive waypoints
        full_path = []
        
        for i in range(len(self.waypoints) - 1):
            start_x, start_y = self.waypoints[i][2:4]
            goal_x, goal_y = self.waypoints[i+1][2:4]
            
            print(f"\n   Segment {i+1}/{len(self.waypoints)-1}: ({start_x:.2f}, {start_y:.2f}) → ({goal_x:.2f}, {goal_y:.2f})")
            
            segment = self.planner.plan(start_x, start_y, goal_x, goal_y)
            
            if segment is None:
                print(f"❌ Failed to plan segment {i+1}!")
                print(f"   Cannot connect waypoint {i+1} to waypoint {i+2}")
                return
            
            # Add segment to full path (avoid duplicating waypoints)
            if i == 0:
                full_path.extend(segment)
            else:
                full_path.extend(segment[1:])  # Skip first point (same as previous segment's last)
        
        self.current_path = full_path
        
        print(f"\n✅ Complete path: {len(full_path)} waypoints")
        
        # Draw full path on display
        for i in range(len(full_path) - 1):
            x1, y1 = full_path[i]
            x2, y2 = full_path[i + 1]
            
            row1, col1 = self.planner.world_to_grid(x1, y1)
            row2, col2 = self.planner.world_to_grid(x2, y2)
            
            cv2.line(self.display, (col1, row1), (col2, row2), (255, 255, 0), 2)
        
        # Redraw waypoints on top
        for i, (row, col, wx, wy) in enumerate(self.waypoints):
            if i == 0:
                color = (255, 0, 0)
            elif i == len(self.waypoints) - 1:
                color = (0, 0, 255)
            else:
                color = (0, 255, 255)
            
            cv2.circle(self.display, (col, row), 8, color, -1)
            cv2.circle(self.display, (col, row), 10, (255, 255, 255), 2)
        
        cv2.imshow('Path Planner - Click waypoints, press F for final, ESC to quit', self.display)
        
        # Save visualization
        cv2.imwrite('path_result.png', self.display)
        print(f"✅ Saved path_result.png")
        
        # Save path JSON
        self.planner.save_path(full_path, 'planned_path.json')
        
        print(f"\n💡 Options:")
        print(f"   • Click anywhere to plan new path")
        print(f"   • Press 'S' to save path with custom filename")
        print(f"   • Press ESC/Q to quit")
    
    def reset(self):
        """Clear waypoints and path"""
        self.waypoints = []
        self.current_path = None
        self.is_final = False
        print(f"\n🔄 Reset! Click to add waypoints, press 'F' for final")
        self.update_display()
    
    def save_current_path(self):
        """Save current path to JSON"""
        if self.current_path:
            filename = input("\nEnter filename (default: planned_path.json): ").strip()
            if not filename:
                filename = 'planned_path.json'
            self.planner.save_path(self.current_path, filename)
        else:
            print("❌ No path to save! Plan a path first.")
    
    def run(self):
        """Run interactive planner"""
        print("=" * 60)
        print("🗺️  MULTI-WAYPOINT PATH PLANNER")
        print("=" * 60)
        print("📍 Click on GREEN areas to add unlimited waypoints")
        print("   • First click = START (blue)")
        print("   • Middle clicks = INTERMEDIATE (yellow)")
        print("   • Press 'F' after clicking FINAL waypoint (turns red)")
        print("")
        print("⌨️  Keyboard controls:")
        print("   • F = Mark final waypoint and plan path")
        print("   • R = Reset (clear all waypoints)")
        print("   • S = Save path with custom filename")
        print("   • ESC/Q = Quit")
        print("=" * 60)
        
        cv2.namedWindow('Path Planner - Click waypoints, press F for final, ESC to quit')
        cv2.setMouseCallback('Path Planner - Click waypoints, press F for final, ESC to quit', 
                            self.mouse_callback)
        
        self.update_display()
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            
            if key == 27 or key == ord('q'):  # ESC or Q
                break
            elif key == ord('f') or key == ord('F'):  # Final
                self.mark_final_and_plan()
            elif key == ord('r') or key == ord('R'):  # Reset
                self.reset()
            elif key == ord('s') or key == ord('S'):  # Save
                self.save_current_path()
        
        cv2.destroyAllWindows()
        print("\n👋 Goodbye!")


if __name__ == '__main__':
    planner = InteractivePlanner()
    planner.run()
