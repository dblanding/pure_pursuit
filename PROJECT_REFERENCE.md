# Project Reference

## Project Overview
* **Goal:** Autonomous mobile robot (AMR) with path planning and following
* **Architecture:** Modular Python programs communicating via MQTT (ROS-inspired, no ROS)
* **Hardware:** [Raspibot](https://github.com/dblanding/raspibot), Raspberry Pi, Pico, RPLIDAR, Sparkfun Optical Tracking Odometer Sensor
 
---
  
## MQTT Topics
**Rule:** No hard-coded topic names. Import from: `from topics import Topics`
 
| Topic Name | Constant | Description | Message Format | Publisher | Subscriber(s) |
|------------|----------|-------------|----------------|-----------|---------------|
| `robot/odometry/raw` | `ODOM_RAW` | Raw odometry from OTOS (odom frame) | `{"x": 0.0, "y": 0.0, "theta": 0.0, "t": 12345.67, "xr": 0.0, "yr": 0.0, "hr": 0.0}` | odometry.py | localization.py |
| `robot/pose` | `POSE` | Localized pose (map frame) | `{"x": 0.70, "y": 3.25, "theta": 0.0, "xr": 0.0, "yr": 0.0, "hr": 0.0}` | localization.py | path_follower.py |
| `robot/initialpose` | `INITIAL_POSE` | Set initial pose | `{"x": 0.70, "y": 3.25, "theta": 0.0}` | User/GUI | localization.py |
| `robot/cmd_vel` | `CMD_VEL` | Velocity commands | `{"linear": 0.2, "angular": 0.5}` | path_follower.py | motor_controller.py |
| `robot/lidar/scan` | `LIDAR_SCAN` | Lidar scan data | `[{"a": -3.14, "d": 0.5, "t": 12345}, ...]` | lidar_node.py | localization.py (future) |

**Message format notes:**
- `x`, `y`, `theta`: Position/orientation
- `t`: Timestamp
- `xr`, `yr`, `hr`: Velocity rates from OTOS IMU (linear_x, linear_y, angular)
---
 
## Coordinate Systems
 
### **Map Frame** (Global)
- Origin: `(-3.0, -7.0)` meters (from map_metadata.json)
- Resolution: `0.05` m/pixel
- Used by: path planner, localized pose
- **Home position:** `(0.70, 3.25, 0.0)` - blue dot parking spot
 
### **Odom Frame** (Local)
- Origin: Where robot was when odometry started
- Always starts at `(0, 0, 0)`
- Used by: raw odometry
- Drifts over time but smooth
 
### **Robot Frame** (Body)
- Origin: Robot center
- +X = forward ✅ (confirmed May 12, 2026)
- +Y = left
- +Theta = counter-clockwise (right-hand rule)
 
### **Transform Chain**

## Platform Notes

### **Ubuntu Linux (Development Laptop)**
- Use `\n` for newlines in print statements (not `\r`)
- Terminal output should use proper line feeds
- Example:
```python
# Good for Ubuntu
print(f"\n✅ Pose updated: ({x:.2f}, {y:.2f})")

# Bad (Windows-style, doesn't work properly)
print(f"\r✅ Pose updated: ({x:.2f}, {y:.2f})", end='')
```

## File Locations
 
### **Configuration (SPOT - Single Point of Truth)**
- `topics.py` - All MQTT topic names
- `map_metadata.json` - Map parameters, home position
- `map_utils.py` - Utilities for reading map metadata
 
### **Data Files**
- `map.pgm` - Original map image
- `occupancy_grid.npy` - Processed occupancy grid for planning
- `map_inflated.npy` - Inflated map with safety margins
- `planned_path.json` - Output from path planner
- `test_x_path.json` - Test path (straight line in +X)
 
### **Programs**
- `motor_controller.py` - Low-level motor control, serial to Pico
- `odometry.py` - Reads OTOS sensor (optical tracking + IMU)
- `localization.py` - Transforms odom to map frame (Phase 1: simple, Phase 2: particle filter)
- `path_planner.py` - A* path planning
- `interactive_planner.py` - GUI for planning paths
- `path_follower.py` - Pure pursuit controller
- `scanner.py` - RPLIDAR interface

### **Odometry**
- **Sensor:** Sparkfun Optical Tracking Odometry Sensor (OTOS)
- **Features:** 
  - Optical tracking (no wheel slip issues)
  - Built-in IMU for velocity rates
  - I2C interface to Raspberry Pi
- **Update rate:** ~20 Hz (verify actual rate)
- **Output:** Position (x, y, theta) + velocities (xr, yr, hr)
- **Frame:** Always starts at (0, 0, 0) in odom frame
- **Advantages:** No wheel slip, no calibration of wheel diameter
---
 
## Key Parameters
 
### **Path Following**
- Lookahead distance: `0.3` m (pure pursuit)
- Waypoint threshold: `0.2` m (when to advance to next waypoint)
- Goal threshold: `0.15` m (when path is complete)
 
### **Velocity Limits**
- Max linear speed: `0.3` m/s (conservative for testing)
- Max angular speed: `1.0` rad/s (~57°/s)
- Emergency stop: `0.0` m/s
 
### **Control Loop**
- Update rate: `10 Hz` (0.1s period)
- Odometry rate: `20 Hz` (0.05s period)
 
### **Map Parameters**
- Inflation radius: `0.15` m (robot safety margin)
- Planning resolution: `0.05` m/cell
 
---
 
## Architecture & Data Flow
```
┌─────────────┐
│  User Input │ (interactive_planner.py)
└──────┬──────┘
       │ planned_path.json
       ↓
┌─────────────┐     ┌──────────────┐
│ Path        │────→│ Path         │
│ Planner     │     │ Follower     │
│ (A*)        │     │ (Pure Pursuit)│
└─────────────┘     └──────┬───────┘
                           │ cmd_vel
                           ↓
                    ┌──────────────┐
                    │ Motor        │
                    │ Controller   │
                    └──────┬───────┘
                           │ Serial to Pico
                           ↓
                    ┌──────────────┐
                    │ Pico + Motors│
                    └──────────────┘

                ┌──────────────┐
                │ OTOS Sensor  │
                │ (optical +   │
                │  IMU)        │
                └──────┬───────┘
                       │ I2C
                       ↓
                ┌──────────────┐
                │  Odometry    │
                │  (reads OTOS)│
                └──────┬───────┘
                       │ odom_raw (odom frame)
                       ↓
                ┌──────────────┐
                │ Localization │
                │ (transforms) │
                └──────┬───────┘
                       │ pose (map frame)
                       └──────────┘
                              │
                              └──→ (feeds back to Path Follower)
```

---

## Current Implementation Status

### **✅ Working**
- Motor control via MQTT
- Odometry from OTOS sensor
- A* path planning
- Interactive path planning GUI
- Pure pursuit path following
- Map inflation for safety

### **🚧 In Progress (Phase 1)**
- Simple localization (odom → map transform)
- Initial pose setting
- **Migration:** ODOM_POSE → ODOM_RAW + POSE split

### **📋 Planned (Phase 2+)**
- Particle filter localization with lidar
- Landmark-based pose correction
- Dynamic obstacle avoidance
- Multi-goal mission planning
---

## Known Issues

### **Fixed**
- ✅ Motor controller subscribed to wrong topic (fixed May 12)
- ✅ Robot turning left at path start (coordinate frame mismatch - fixed with localization)

### **Active**
- [ ] None currently

### **Future Considerations**
- Odometry drift over long distances (will be addressed by particle filter)
- OTOS sensor calibration/zeroing procedure

---

## Testing Procedures

### **Test 1: Motor Direction**
```bash
# Verify forward = +X direction
mosquitto_pub -h localhost -t 'robot/cmd_vel' -m '{"linear": 0.2, "angular": 0.0}'
# Robot should move forward in +X direction
```

### Test 2: Odometry
```bash
# Monitor odometry while manually moving robot
mosquitto_sub -h localhost -t 'robot/odometry/raw' -v
# Should start at (0, 0, 0) and update smoothly
```

### Test 3: Localization
```bash
# Set initial pose and verify transform
mosquitto_pub -h localhost -t 'robot/initialpose' -m '{"x": 0.70, "y": 3.25, "theta": 0.0}'
mosquitto_sub -h localhost -t 'robot/pose' -v
# Should show pose in map frame
```

### Test 4: Path Following
```bash
# Simple straight-line test
uv run path_follower.py --path-file test_x_path.json
# Robot should follow path without initial turn
```

---

## Startup Sequence
Normal Operation (from home position)
```bash
# 1. Start core services
uv run motor_controller.py &
uv run odometry.py &
uv run localization.py &

# 2. Park robot at home (blue dot)

# 3. Set initial pose
mosquitto_pub -h localhost -t 'robot/initialpose' -m '{"x": 0.70, "y": 3.25, "theta": 0.0}'

# 4. Plan path
uv run interactive_planner.py --from-home
# Click goal position

# 5. Execute path
uv run path_follower.py --path-file planned_path.json
```

## Debug Commands
```bash
# Monitor all topics
mosquitto_sub -h localhost -t 'robot/#' -v

# Check specific topic
mosquitto_sub -h localhost -t 'robot/pose' -v

# Manual velocity command
mosquitto_pub -h localhost -t 'robot/cmd_vel' -m '{"linear": 0.1, "angular": 0.0}'

# Emergency stop
mosquitto_pub -h localhost -t 'robot/cmd_vel' -m '{"linear": 0.0, "angular": 0.0}'

# Set initial pose to home
mosquitto_pub -h localhost -t 'robot/initialpose' -m '{"x": 0.70, "y": 3.25, "theta": 0.0}'
```

## Dependencies
```
# Python packages (from pyproject.toml)
paho-mqtt       # MQTT communication
numpy           # Numerical operations
opencv-python   # Image processing, GUI
scipy           # Path smoothing (if used)

# System
mosquitto       # MQTT broker
```

## Learning Resources

### Concepts Used
* Pure pursuit path following
* A* pathfinding
* Occupancy grid mapping
* Coordinate frame transforms
* Odometry integration

### Future Topics
* Particle filter (Monte Carlo Localization)
* AprilTag detection
* Kalman filtering
* Dynamic Window Approach (DWA)

### Recent Findings & Decisions
* May 12, 2026:

    * ✅ Confirmed robot +X = forward direction
    * ✅ Implemented localization architecture (Phase 1)
    * ✅ Decided to follow ROS patterns without using ROS
    * ✅ Established SPOT principle for configuration (map_metadata.json)
    * 🎓 Student goal: Build proper foundation for future particle filter

### Notes
* This is a learning project - prioritizing understanding over shortcuts
* Architecture inspired by ROS but implemented standalone
* Code style: Clear over clever, educational over optimal
* Following "Pure Pursuit 101" curriculum 🎓

