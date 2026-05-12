# Project Reference

## MQTT Topics
- **cmd_vel**: `robot/cmd_vel` - Motor commands (JSON: `{"linear": float, "angular": float}`)
- **mode**: `robot/mode` - Mode switching (String: "AUTO" or "MANUAL")
- **pose/odometry**: `???` - Robot position (FILL THIS IN)
- **Other topics**: (add as needed)

## Coordinate System
- Robot forward = +X direction ✅ (confirmed May 12, 2026)
- Map origin: (-3.0, -7.0)
- Map resolution: 0.05 m/cell

## File Locations
- Map: `map.pgm`
- Occupancy grid: `occupancy_grid.npy`
- Path files: `planned_path.json`, etc.

## Key Parameters
- Waypoint threshold: `???` (FILL THIS IN)
- Max linear speed: 3.2 M/s
- Max angular speed: `???`

## Components
- `motor_controller.py` - Subscribes to cmd_vel, controls motors
- `path_planner.py` - A* planning, PathPlanner class
- `interactive_planner.py` - GUI for clicking waypoints
- `path_follower.py` - Follows planned paths
- `localization.py` - Particle filter (if exists?)

## Known Issues
- Path follower may not be getting robot position updates
- Robot "kept going" without stopping at waypoint

## Recent Findings
- Motor controller was subscribed to wrong topic initially (fixed)
- Test showed robot moves in +X when commanded forward
