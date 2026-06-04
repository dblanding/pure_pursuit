#!/usr/bin/env python3
# obstacle_avoidance.py
"""
VFH-based obstacle avoidance safety node.

Sits between path_follower and the motor controller:

    path_follower  →  Topics.NAV_CMD_VEL  →  [this script]  →  Topics.MOTOR_CMD
                                                     ↑
                                              Topics.LIDAR_SCAN
                                              Topics.POSE

Normal operation (path clear):
    Commands from path_follower are passed straight through to the motors
    with no modification.

Obstacle detected:
    If any scan point falls within the DANGER zone (very close, within
    DANGER_DIST_M of the robot centre), a STOP command is published
    immediately and path_follower commands are suppressed until the
    obstacle clears.

    If scan points are only within the WARNING zone (between DANGER_DIST_M
    and WARNING_DIST_M), the linear velocity is scaled down proportionally
    so the robot slows as it approaches, but does not stop.

The detection arc is limited to the FORWARD_ARC_DEG sector centred on the
robot's forward direction so the robot can still turn away from an obstacle
without triggering a false stop.

Run this script BEFORE path_follower.py.  path_follower must be configured
to publish to Topics.NAV_CMD_VEL (not Topics.MOTOR_CMD directly).

Usage:
    python obstacle_avoidance.py
    python obstacle_avoidance.py --broker 192.168.1.85
"""

import sys
import json
import math
import time
import threading
import argparse
import paho.mqtt.client as mqtt

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_BROKER  = "raspibot.local"
BROKER_PORT     = 1883
MQTT_USERNAME   = "robot"
MQTT_PASSWORD   = "robot"

# Detection arc — only scan points within this angular window ahead of the
# robot are considered.  60° means ±30° either side of forward.
FORWARD_ARC_DEG = 60.0

# Distance thresholds (metres from robot centre)
DANGER_DIST_M  = 0.30   # inside this → STOP immediately
WARNING_DIST_M = 0.60   # inside this → scale down linear velocity

# Maximum scan range to consider (ignore far-away returns)
MAX_SCAN_RANGE = 3.0

# Minimum number of scan points that must be inside the danger/warning zone
# before triggering a response.  Prevents single noisy returns causing stops.
MIN_OBSTACLE_POINTS = 3

# How long (seconds) to keep suppressing commands after an obstacle clears.
# Gives the robot time to react before resuming full speed.
CLEAR_HOLDOFF_SEC = 0.5

# Watchdog: if no command arrives from path_follower for this many seconds,
# publish a stop (path_follower may have crashed).
CMD_WATCHDOG_SEC = 1.0

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
latest_scan  = None
latest_pose  = None
latest_cmd   = None          # most recent NAV_CMD_VEL from path_follower
last_cmd_t   = 0.0           # timestamp of latest_cmd
obstacle_t   = 0.0           # time obstacle was last seen
lock         = threading.Lock()


# ---------------------------------------------------------------------------
# Scan analysis
# ---------------------------------------------------------------------------

def scan_to_cartesian_robot_frame(scan):
    """
    Convert scan to robot-frame (x=forward, y=left) Cartesian points.
    Filters to MAX_SCAN_RANGE.  Returns (N,2) list of [x, y].
    """
    pts = []
    for entry in scan:
        a = entry["a"]          # CCW positive, 0 = east = robot forward
        d = entry["d"]
        if 0.05 < d < MAX_SCAN_RANGE:
            x = d * math.cos(a)
            y = d * math.sin(a)
            pts.append((x, y))
    return pts


def analyse_obstacles(scan):
    """
    Analyse scan for obstacles in the forward arc.

    Returns:
        (state, scale, n_danger, n_warning, closest_m)

        state:   'clear' | 'warning' | 'danger'
        scale:   velocity scale factor to apply (1.0 = full, 0.0 = stop)
        n_danger, n_warning: obstacle point counts
        closest_m: distance to nearest obstacle point in arc (or inf)
    """
    pts     = scan_to_cartesian_robot_frame(scan)
    half    = math.radians(FORWARD_ARC_DEG / 2.0)

    n_danger  = 0
    n_warning = 0
    closest   = float("inf")

    for x, y in pts:
        # Bearing from robot forward (+x axis)
        bearing = math.atan2(y, x)    # CCW positive
        if abs(bearing) > half:
            continue                   # outside forward arc

        dist = math.hypot(x, y)
        if dist < closest:
            closest = dist

        if dist < DANGER_DIST_M:
            n_danger += 1
        elif dist < WARNING_DIST_M:
            n_warning += 1

    if n_danger >= MIN_OBSTACLE_POINTS:
        return "danger", 0.0, n_danger, n_warning, closest

    if n_warning >= MIN_OBSTACLE_POINTS:
        # Scale linearly: full speed at WARNING_DIST_M, zero at DANGER_DIST_M
        scale = (closest - DANGER_DIST_M) / (WARNING_DIST_M - DANGER_DIST_M)
        scale = max(0.0, min(1.0, scale))
        return "warning", scale, n_danger, n_warning, closest

    return "clear", 1.0, 0, 0, closest


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.LIDAR_SCAN)
        client.subscribe(Topics.NAV_CMD_VEL)
        print(f"Connected to broker.")
        print(f"  Listening:  {Topics.LIDAR_SCAN}")
        print(f"              {Topics.NAV_CMD_VEL}")
        print(f"  Forwarding: {Topics.MOTOR_CMD}")
        print(f"  Danger zone:  {DANGER_DIST_M:.2f}m  "
              f"Warning zone: {WARNING_DIST_M:.2f}m  "
              f"Arc: ±{FORWARD_ARC_DEG/2:.0f}°")
    else:
        print(f"Connection failed rc={rc}")


def on_scan(client, userdata, msg):
    global latest_scan
    with lock:
        latest_scan = json.loads(msg.payload.decode())


def on_nav_cmd(client, userdata, msg):
    """Receive velocity request from path_follower."""
    global latest_cmd, last_cmd_t
    with lock:
        latest_cmd  = json.loads(msg.payload.decode())
        last_cmd_t  = time.monotonic()


def on_message(client, userdata, msg):
    if msg.topic == Topics.LIDAR_SCAN:
        on_scan(client, userdata, msg)
    elif msg.topic == Topics.NAV_CMD_VEL:
        on_nav_cmd(client, userdata, msg)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def make_stop_cmd():
    return json.dumps({"linear": 0.0, "angular": 0.0,
                       "timestamp": time.time()})


def run(broker):
    global obstacle_t

    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, BROKER_PORT, 60)
    client.loop_start()

    print(f"Obstacle avoidance starting — connecting to {broker}...")
    prev_state  = "clear"
    loop_rate   = 0.05   # 20 Hz decision loop

    try:
        while True:
            time.sleep(loop_rate)
            now = time.monotonic()

            with lock:
                scan = latest_scan
                cmd  = latest_cmd
                cmd_age = now - last_cmd_t

            # Watchdog: stop if path_follower has gone silent
            if cmd is not None and cmd_age > CMD_WATCHDOG_SEC:
                client.publish(Topics.MOTOR_CMD, make_stop_cmd())
                if prev_state != "watchdog":
                    print(f"⚠️  Watchdog: no command for "
                          f"{cmd_age:.1f}s — stopping")
                prev_state = "watchdog"
                continue

            if scan is None or cmd is None:
                continue    # not ready yet

            # Analyse obstacles
            state, scale, n_danger, n_warning, closest = analyse_obstacles(scan)

            # Track last time obstacle was seen
            if state != "clear":
                obstacle_t = now

            # Apply holdoff after obstacle clears
            in_holdoff = (now - obstacle_t) < CLEAR_HOLDOFF_SEC

            if state == "danger" or (state == "clear" and in_holdoff
                                     and prev_state == "danger"):
                # STOP
                client.publish(Topics.MOTOR_CMD, make_stop_cmd())
                if prev_state != "danger":
                    print(f"🛑 STOP: {n_danger} points within "
                          f"{DANGER_DIST_M:.2f}m  "
                          f"(closest={closest:.3f}m)")
                prev_state = "danger"

            elif state == "warning" or in_holdoff:
                # Scale down linear, pass angular unchanged
                modified = dict(cmd)
                modified["linear"]    = round(cmd["linear"] * scale, 4)
                modified["timestamp"] = time.time()
                client.publish(Topics.MOTOR_CMD, json.dumps(modified))
                if prev_state not in ("warning",):
                    print(f"⚠️  WARNING: {n_warning} points in "
                          f"{DANGER_DIST_M:.2f}-{WARNING_DIST_M:.2f}m  "
                          f"(closest={closest:.3f}m  scale={scale:.2f})")
                prev_state = "warning"

            else:
                # Clear — pass command straight through
                fwd = dict(cmd)
                fwd["timestamp"] = time.time()
                client.publish(Topics.MOTOR_CMD, json.dumps(fwd))
                if prev_state != "clear":
                    print(f"✅ Path clear (closest={closest:.3f}m)")
                prev_state = "clear"

    except KeyboardInterrupt:
        print("\nObstacle avoidance stopped.")
        client.publish(Topics.MOTOR_CMD, make_stop_cmd())
    finally:
        client.loop_stop()
        client.disconnect()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="VFH obstacle avoidance safety node")
    parser.add_argument("--broker", default=DEFAULT_BROKER,
                        help=f"MQTT broker (default: {DEFAULT_BROKER})")
    args = parser.parse_args()
    run(args.broker)
