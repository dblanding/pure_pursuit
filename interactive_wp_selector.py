#!/usr/bin/env python3
"""
Interactive waypoint selector - click unlimited waypoints
Press 'F' to mark final waypoint and plan path
"""
import cv2
import numpy as np
from path_planner import PathPlanner

# Display scale factor — change to 3 for 3x size
SCALE = 3

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

    def _scaled(self, img):
        """Return a scaled copy of img for display."""
        if SCALE == 1:
            return img
        h, w = img.shape[:2]
        return cv2.resize(img, (w * SCALE, h * SCALE), interpolation=cv2.INTER_NEAREST)

    def update_display(self):
        """Redraw display with current waypoints."""
        # Reset to base map
        self.display = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        free_mask = self.map_img > 127
        self.display[free_mask] = [0, 200, 0]

        # Draw waypoints (coordinates in original map space)
        for i, (row, col, wx, wy) in enumerate(self.waypoints):
            if i == 0:
                color = (255, 0, 0)       # Start = blue
            elif i == len(self.waypoints) - 1 and self.is_final:
                color = (0, 0, 255)       # Final = red
            else:
                color = (0, 255, 255)     # Intermediate = yellow

            cv2.circle(self.display, (col, row), 8, color, -1)
            cv2.circle(self.display, (col, row), 10, (255, 255, 255), 2)
            cv2.putText(self.display, str(i + 1), (col + 12, row + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Draw connector lines between waypoints
        if len(self.waypoints) > 1:
            for i in range(len(self.waypoints) - 1):
                row1, col1 = self.waypoints[i][:2]
                row2, col2 = self.waypoints[i + 1][:2]
                cv2.line(self.display, (col1, row1), (col2, row2), (128, 128, 128), 2)

        cv2.imshow(self._win_name(), self._scaled(self.display))

    def _win_name(self):
        return 'Path Planner - Click waypoints, press F for final, ESC to quit'

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks — x/y are in scaled screen space."""
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        # Convert from scaled screen coords back to original map coords
        map_x = x // SCALE
        map_y = y // SCALE

        # If already planned, reset
        if self.current_path is not None:
            print(f"\n🔄 Resetting for new path")
            self.reset()

        # Bounds check
        if map_y < 0 or map_y >= self.map_img.shape[0] or \
           map_x < 0 or map_x >= self.map_img.shape[1]:
            print(f"❌ Click outside map bounds")
            return

        if self.map_img[map_y, map_x] < 127:
            print(f"❌ Clicked on obstacle at ({map_x}, {map_y})")
            return

        # Convert to world coordinates
        world_x, world_y = self.planner.grid_to_world(map_y, map_x)

        # Add waypoint
        self.waypoints.append((map_y, map_x, world_x, world_y))
        self.is_final = False

        print(f"✅ Waypoint {len(self.waypoints)}: grid({map_y}, {map_x}) = world({world_x:.2f}, {world_y:.2f})")

        if len(self.waypoints) == 1:
            print(f"   💡 Keep clicking to add more waypoints")
            print(f"   💡 Press 'F' when you click the FINAL waypoint")

        self.update_display()

    def mark_final_and_plan(self):
        """Mark last waypoint as final and plan path."""
        if len(self.waypoints) < 2:
            print(f"❌ Need at least 2 waypoints! (have {len(self.waypoints)})")
            print(f"   💡 Click more waypoints first")
            return

        self.is_final = True
        self.update_display()  # Redraw with final waypoint in red

        print(f"\n🎯 Final waypoint marked!")
        print(f"🚀 Planning path through {len(self.waypoints)} waypoints...")

        full_path = []

        for i in range(len(self.waypoints) - 1):
            start_x, start_y = self.waypoints[i][2:4]
            goal_x, goal_y = self.waypoints[i + 1][2:4]

            print(f"\n   Segment {i+1}/{len(self.waypoints)-1}: ({start_x:.2f}, {start_y:.2f}) → ({goal_x:.2f}, {goal_y:.2f})")

            segment = self.planner.plan(start_x, start_y, goal_x, goal_y)

            if segment is None:
                print(f"❌ Failed to plan segment {i+1}!")
                print(f"   Cannot connect waypoint {i+1} to waypoint {i+2}")
                return

            if i == 0:
                full_path.extend(segment)
            else:
                full_path.extend(segment[1:])

        self.current_path = full_path
        print(f"\n✅ Complete path: {len(full_path)} waypoints")

        # Draw full path (in original map space)
        for i in range(len(full_path) - 1):
            x1, y1 = full_path[i]
            x2, y2 = full_path[i + 1]
            row1, col1 = self.planner.world_to_grid(x1, y1)
            row2, col2 = self.planner.world_to_grid(x2, y2)
            cv2.line(self.display, (col1, row1), (col2, row2), (255, 255, 0), 2)

        # Redraw waypoints on top of path
        for i, (row, col, wx, wy) in enumerate(self.waypoints):
            color = (255, 0, 0) if i == 0 else (0, 0, 255) if i == len(self.waypoints) - 1 else (0, 255, 255)
            cv2.circle(self.display, (col, row), 8, color, -1)
            cv2.circle(self.display, (col, row), 10, (255, 255, 255), 2)

        cv2.imshow(self._win_name(), self._scaled(self.display))

        # Save outputs (at original resolution — no scaling needed for files)
        cv2.imwrite('path_result.png', self.display)
        print(f"✅ Saved path_result.png")
        self.planner.save_path(full_path, 'planned_path.json')

        print(f"\n💡 Options:")
        print(f"   • Click anywhere to plan new path")
        print(f"   • Press 'S' to save path with custom filename")
        print(f"   • Press ESC/Q to quit")

    def reset(self):
        """Clear waypoints and path."""
        self.waypoints = []
        self.current_path = None
        self.is_final = False
        print(f"\n🔄 Reset! Click to add waypoints, press 'F' for final")
        self.update_display()

    def save_current_path(self):
        """Save current path to JSON."""
        if self.current_path:
            filename = input("\nEnter filename (default: planned_path.json): ").strip()
            if not filename:
                filename = 'planned_path.json'
            self.planner.save_path(self.current_path, filename)
        else:
            print("❌ No path to save! Plan a path first.")

    def run(self):
        """Run interactive planner."""
        print("=" * 60)
        print("🗺️  MULTI-WAYPOINT PATH PLANNER")
        print("=" * 60)
        print(f"🔍 Display scale: {SCALE}x  (edit SCALE at top of file to change)")
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

        cv2.namedWindow(self._win_name())
        cv2.setMouseCallback(self._win_name(), self.mouse_callback)

        self.update_display()

        while True:
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord('q'):   # ESC or Q
                break
            elif key == ord('f') or key == ord('F'):
                self.mark_final_and_plan()
            elif key == ord('r') or key == ord('R'):
                self.reset()
            elif key == ord('s') or key == ord('S'):
                self.save_current_path()

        cv2.destroyAllWindows()
        print("\n👋 Goodbye!")


if __name__ == '__main__':
    planner = InteractivePlanner()
    planner.run()
