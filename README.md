# Pure Pursuit

## Starts with a decent map
* I used MS Paint to clean up map.png (from OGmapper) and saved it as map_clean.png
* Then went through these steps:
    1. Convert to Binary Planning Map
        * Run it through `create_planning_map.py`
    2. Create Map Metadata File
        * `create_metadata.py`
    3. Create A Path Planner*
        * Run it using `uv run path_planner.py`
* It turns out that MS Paint insidiously converted my file to a **2.7x** higher resolution.
    * Use `check_res.py` to compare resolution before & after cleanup.
    * Paint resized it from 400×400 to 1079×1039.
* Pro tip for the future: Use these tools instead
    * GIMP (free, doesn't resize accidentally)
    * ImageMagick (`convert map.png -threshold 50% map_clean.png`)

## Fix map: Resize Back to 400×400
* `uv run resize_map.py`
* Repeat steps above:
    1. Recreate planning map with correct dimensions `uv run create_planning_map.py`
    2. Test path planning again `uv run path_planner.py`

## Inflation
* `check_inflation.py` generates a pair of images showing the effect of inflation

## Interactive Goal Selector
* **interactive_planner.py** is a *front-end* for path_planner.py that makes it easy to pick valid start/goal points by clicking on a map. Run it with `uv run interactive_planner.py`
* How to Use:
    1. Window opens showing your map (green = safe, black = obstacles)
    2. Click on green area → Sets start (blue circle)
    3. Click another green area → Sets goal (red circle) and automatically plans path
    4. Cyan line shows the path
    5. Click again → Plans new path with new start/goal
 ## Add *Path Smoothing*
* The A* path has jagged stair-step movements because it moves cell-by-cell on a grid. Smooothing removes unneccesary waypoints and draws straight lines where possible.
* Add smoothing to *path_planner.py*
 
## Now we're Ready to Drive! 🚗
* For starters, let's plan on operating in "Home Base" mode 🏠, where each trip will originate at pose (0, 0, 0)
* Architecture Overview:
```
┌─────────────────────────────────────────────┐
│  RobotNavigator                             │
│  - Manages missions                         │
│  - Plans paths                              │
│  - Coordinates return to home               │
└─────────────────┬───────────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼────────┐  ┌───────▼────────┐
│ PathPlanner    │  │ PathFollower   │
│ ( have this)   │  │ (build this)   │
└────────────────┘  └───────┬────────┘
                            │
                    ┌───────▼────────┐
                    │ Odometry       │
                    │ (encoders+IMU) │
                    └────────────────┘
```
* Create files:
    * *path_follower.py* converts waypoints into velocity commands
    * *robot_navigator.py* high-level mission controller
    * *robot_interface.py* stub for connecting to actual hardware

## Pause to consider how this will integrate into my existing robot control system.
![My Desktop Control System](imgs/IMG_4860.jpeg)
* My setup includes the following:
    * An onboard mqtt broker that publishes (currently) both lidar scans and pose.
    * I have a ssh connected GUI for monitoring and stopping and starting services
    * A web-based MQTT monitor
    * A joystick control (connected via BLE) for driving the robot in teleop mode
    * Robot accepts drive commands with the format: (lin_spd, ang_spd)
* I would like to have all my path planning and path following programs on my laptop.
    * *path_follower.py* on laptop publishes motor drive commands on robot/motor/cmd topic
* I would like to have an mqtt listener on my robot subscribing to motor drive commands.
    * *motor_control_service.py* on robot subscribes to motor commands and drives the motors.
* The GUI would have a couple of buttons to start and stop the motor_control service.

