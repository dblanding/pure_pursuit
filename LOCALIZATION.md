# Localization

## Here's my existing robot control system.
![My Desktop Control System](imgs/IMG_4860.jpeg)
* My setup includes the following:
    * An onboard mqtt broker that publishes (currently) both lidar scans and pose.
    * I have a ssh connected GUI for monitoring and stopping and starting services
    * A web-based MQTT monitor
    * 2 ways to drive motors:
        1. A joystick control (connected via BLE) in teleop mode 
        2. Robot accepts MQTT drive commands with the format: (lin_spd, ang_spd)

## Thinking about how the robot will make pose corrections when it drifts over time.

* This will likely require an intermediate layer between the robot and the world coordinate frame
* Thinking through a Clean Robotics Architecture for accomplishing this.
* We're going to build a ROS-like architecture, but without ROS

    * What we're borrowing from ROS (the good ideas):

        - ✅ Separation of odometry vs. localization
        - ✅ Coordinate frame concepts (map vs. odom)
        - ✅ initialpose message pattern
        - ✅ Modular programs communicating via MQTT (like ROS nodes via topics)

    * What we're NOT using:

        - ❌ ROS nodes, roslaunch, roscore
        - ❌ ROS tf2 transform library
        - ❌ ROS message types (.msg files)
        - ❌ ROS-specific terminology in your code

* In our new clean architecture, we will have Programs (not "nodes"):
```
odometry.py          → Tracks wheel movement (odom frame)
localization.py      → Figures out position on map (map frame)
path_planner.py      → Creates paths in map coordinates
path_follower.py     → Follows paths using localized position
motor_controller.py  → Low-level motor control
```

* Whereas ROS uses these 3 coordinate frames:
    1. Map frame - Global, fixed coordinate system
        * Origin at map corner or some fixed point
        * Where landmarks, walls, goals exist
        * Never changes

    2. Odom frame - Local, continuous coordinate system  
        * Origin where robot started (or last reset)
        * Tracks relative motion from encoders/IMU
        * Drifts over time but smooth

    3. Base_link frame - Robot's body
        * Origin at robot center
        * Moves with the robot

* We will refer to them this way (and we won't be using ROS tf):
    1. Map frame:    Global coordinates (your map_metadata.json defines this)
    2. Odom frame:   Local coordinates (where odometry starts at 0,0,0)
    3. Robot frame:  Robot's perspective (forward = +X in robot's view)

* In ROS, Transform is used to translate from one coordinate frame to another, whereas we will use LOCALIZATION
```
map → odom → base_link
    ↑
    └─ This transform comes from LOCALIZATION
```

* For Communication between programs:
    * We will use MQTT topics (not the same as ROS topics, but same idea)
    * We will use JSON messages (not ROS .msg types)

## Implementing the new architecture:

#### Phase 1 (this week): Simple localization
```
# localization.py - Version 1.0 (no particle filter yet)
# Just transforms odometry to map frame using initial pose
```
* This gives you:

    Proper architecture
    Odometry stays clean (always starts at 0,0,0)
    Easy to add particle filter later
    Only ~50 lines of code

#### Phase 2 (later): Add particle filter
```
# localization.py - Version 2.0
# Adds particle filter that uses lidar to correct drift
```

