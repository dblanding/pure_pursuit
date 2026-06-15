#!/usr/bin/env python3
# wall_follower.py
"""
Pure-lidar wall-following exploration.

Reactive alternative to frontier-based exploration (frontier_explorer.py).
Instead of selecting distant goals and navigating to them, the robot
simply follows a wall at a fixed standoff distance — building up the map
(via icp_localizer.py running alongside, unchanged) as it goes.

Algorithm:
  - Look at the lidar arc on the chosen side (default: right, ~-60° to -120°
    in math convention where 0°=fwd, +CCW=left, -CW=right).
  - PD-control angular velocity to hold that side's wall at TARGET_DIST.
  - Drive forward at a steady speed.
  - Outside corner (wall distance jumps up — wall fell away): turn toward
    the wall side and sweep until the wall is reacquired.
  - Inside corner (something blocks the front arc within DANGER_DIST):
    stop forward motion, pivot AWAY from the wall side until the front
    clears, then resume.

This script publishes directly to Topics.MOTOR_CMD, bypassing
obstacle_avoidance.py.  Wall-following deliberately operates near VFH's
WARNING_DIST_M threshold, so VFH's STEER logic would constantly fight the
wall-follower's intentional proximity to the wall.  A simple self-contained
front-danger check is more appropriate here.

Usage:
    python wall_follower.py
    python wall_follower.py --side left
    python wall_follower.py --target-dist 0.35 --speed 0.18
"""

import sys
import json
import math
import time
import argparse
import threading
from collections import deque

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

TARGET_DIST     = 0.40    # metres — desired standoff from the followed wall
SPEED           = 0.18    # m/s — forward speed while following
MAX_ANGULAR     = 1.0     # rad/s — angular velocity clamp

# Two-ray side sampling: a ray leaning toward the front (ANGLE_A) and one
# leaning toward the rear (ANGLE_B), both on the wall side, symmetric
# around the perpendicular (90°). Both given as the "side" arc; the
# actual robot-frame angle used is side*ANGLE_A / side*ANGLE_B.
#
# If the robot is parallel to the wall, d_A == d_B (both equal
# D_perp / sin(ANGLE_A), since sin(ANGLE_A)==sin(ANGLE_B) when
# ANGLE_B = 180-ANGLE_A). If the robot's heading drifts toward the wall
# (converging), d_A < d_B; if it drifts away (diverging), d_A > d_B.
# This gives a direct heading-error signal, independent of distance —
# see the geometric derivation that established the sign convention
# heading_term = side*(d_A - d_B).
ANGLE_A         = 70.0    # degrees, front-leaning ray
ANGLE_B         = 110.0   # degrees, rear-leaning ray
RAY_WINDOW_DEG  = 10.0     # +/- window for distance_at_angle
SIN_ANGLE_A     = math.sin(math.radians(ANGLE_A))  # ~0.940; for D_est

# Front-danger arc: stop forward motion if anything is closer than
# DANGER_DIST within this arc (±FRONT_ARC_DEG/2 of straight ahead).
FRONT_ARC_DEG   = 40.0
DANGER_DIST     = 0.30    # metres

# Outside-corner detection: if the estimated perpendicular distance to
# the wall exceeds this, the wall has "fallen away" — turn toward it to
# follow the corner.
LOST_WALL_DIST  = 0.80    # metres

# Gains for the two-ray controller.
#   angular = side * (KP*dist_error + KD*dist_error_deriv + KH*heading_term)
# KP/KD act on perpendicular-distance error (same role as before).
# KH acts on the heading-error term (d_A - d_B) — this is the new term
# that should prevent the "arc back at near-perpendicular incidence"
# cycle, by correcting heading drift before distance error builds up.
KP              = 2.5
KD              = 1.5
KH              = 3.0

# side_d smoothing: average over the last SMOOTH_WINDOW valid readings.
# Reduces oscillation/bumps caused by noisy single-scan readings (e.g.
# thin chair legs producing a brief "wall lost" gap between legs).
# A reading is only treated as truly "lost" after NONE_RESET_COUNT
# consecutive invalid (None) readings — a single noisy gap won't trigger
# a false outside-corner sweep, but a genuine corner still clears the
# buffer quickly.
SMOOTH_WINDOW    = 4
NONE_RESET_COUNT = 2

# Minimum/maximum valid lidar range to consider
MIN_RANGE       = 0.05
MAX_RANGE       = 4.0

STATUS_INTERVAL = 5.0     # seconds between status prints

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
latest_scan    = None
new_scan_ready = False
lock           = threading.Lock()


# ---------------------------------------------------------------------------
# Lidar helpers
# ---------------------------------------------------------------------------

def _norm_deg(a_deg):
    """Normalize an angle in degrees to (-180, 180]."""
    a = a_deg % 360.0
    if a > 180.0:
        a -= 360.0
    return a


def distance_at_angle(scan, target_deg, window_deg=10.0):
    """
    Minimum range within window_deg of target_deg (robot-frame angle,
    math convention: 0=fwd, +CCW=left, -CW=right).
    Returns None if no valid points in the window.
    """
    best = None
    for entry in scan:
        a_deg = _norm_deg(math.degrees(entry["a"]))
        d = entry["d"]
        if d < MIN_RANGE or d > MAX_RANGE:
            continue
        diff = _norm_deg(a_deg - target_deg)
        if abs(diff) <= window_deg:
            if best is None or d < best:
                best = d
    return best


def front_min_distance(scan):
    """Minimum range within the forward danger arc (±FRONT_ARC_DEG/2)."""
    half = FRONT_ARC_DEG / 2.0
    best = None
    for entry in scan:
        a_deg = _norm_deg(math.degrees(entry["a"]))
        d = entry["d"]
        if d < MIN_RANGE or d > MAX_RANGE:
            continue
        if abs(a_deg) <= half:
            if best is None or d < best:
                best = d
    return best


# ---------------------------------------------------------------------------
# MQTT
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.LIDAR_SCAN)
        print(f"Connected to broker.")
        print(f"  Listening:  {Topics.LIDAR_SCAN}")
        print(f"  Publishing: {Topics.MOTOR_CMD}")
    else:
        print(f"Connection failed rc={rc}")


def on_message(client, userdata, msg):
    global latest_scan, new_scan_ready
    try:
        payload = json.loads(msg.payload.decode())
        with lock:
            if msg.topic == Topics.LIDAR_SCAN:
                latest_scan = payload
                new_scan_ready = True
    except Exception as e:
        print(f"Parse error: {e}")


def publish_cmd(client, linear, angular):
    cmd = {"linear": round(float(linear), 4),
           "angular": round(float(angular), 4),
           "timestamp": time.time()}
    client.publish(Topics.MOTOR_CMD, json.dumps(cmd))


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(broker, side_name, target_dist, speed):
    global new_scan_ready

    side = -1 if side_name == "right" else +1
    print(f"Wall-following mode: {side_name} wall, "
          f"target_dist={target_dist}m, speed={speed}m/s")

    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, BROKER_PORT, 60)
    client.loop_start()

    print(f"Connecting to {broker}...")

    prev_dist_error = 0.0
    prev_t       = time.monotonic()
    last_status  = time.monotonic()

    # State for outside-corner sweep: when the wall is lost, turn toward
    # the wall side until it's reacquired or a timeout elapses.
    sweeping     = False
    sweep_start  = 0.0
    SWEEP_TIMEOUT = 3.0    # seconds — give up sweep and go straight

    # State for inside-corner pivot
    pivoting     = False

    # Corner-recovery lock: when front-danger and "wall lost" occur
    # simultaneously (an outside corner approached obliquely presents
    # its end-face as a front obstacle while the long wall has already
    # fallen out of the side rays), a single-step pivot isn't enough —
    # the very next iteration the "wall lost" sweep turns the robot
    # right back toward the end-face, fighting the pivot.  Lock into a
    # longer pivot-away to actually clear the corner before
    # re-evaluating.
    pivot_lock_until    = 0.0
    CORNER_RECOVERY_SEC = 1.5   # seconds — locked pivot-away duration

    # Smoothing buffers for the two rays (each ray smoothed independently
    # over the last SMOOTH_WINDOW valid readings; a single noisy gap on
    # one ray won't immediately trigger "wall lost").
    d_a_buf          = deque(maxlen=SMOOTH_WINDOW)
    d_b_buf          = deque(maxlen=SMOOTH_WINDOW)
    consecutive_none_a = 0
    consecutive_none_b = 0

    try:
        while True:
            time.sleep(0.05)
            now = time.monotonic()

            with lock:
                scan  = latest_scan
                ready = new_scan_ready
                if ready:
                    new_scan_ready = False

            if scan is None:
                continue
            if not ready:
                continue

            dt = max(now - prev_t, 1e-3)
            prev_t = now

            front_d = front_min_distance(scan)

            # Two rays on the wall side: front-leaning (A) and
            # rear-leaning (B), symmetric around perpendicular.
            d_a_raw = distance_at_angle(scan, side * ANGLE_A, RAY_WINDOW_DEG)
            d_b_raw = distance_at_angle(scan, side * ANGLE_B, RAY_WINDOW_DEG)

            # Smooth each ray independently over the last SMOOTH_WINDOW
            # valid readings.  A single noisy gap on one ray (e.g.
            # between thin chair legs) gets averaged out rather than
            # immediately triggering "wall lost".  Genuine wall loss on
            # a ray is detected once NONE_RESET_COUNT consecutive
            # readings are invalid, which clears that ray's buffer.
            if d_a_raw is not None:
                d_a_buf.append(d_a_raw)
                consecutive_none_a = 0
            else:
                consecutive_none_a += 1
                if consecutive_none_a >= NONE_RESET_COUNT:
                    d_a_buf.clear()

            if d_b_raw is not None:
                d_b_buf.append(d_b_raw)
                consecutive_none_b = 0
            else:
                consecutive_none_b += 1
                if consecutive_none_b >= NONE_RESET_COUNT:
                    d_b_buf.clear()

            d_a = (sum(d_a_buf) / len(d_a_buf)) if d_a_buf else None
            d_b = (sum(d_b_buf) / len(d_b_buf)) if d_b_buf else None

            # Determine "wall lost" up front (either ray missing, or
            # the perpendicular-distance estimate exceeds LOST_WALL_DIST)
            d_est = None
            if d_a is None or d_b is None:
                wall_lost = True
            else:
                d_est = ((d_a + d_b) / 2.0) * SIN_ANGLE_A
                wall_lost = d_est > LOST_WALL_DIST

            # --- Corner-recovery lock: keep pivoting away regardless of
            # this iteration's readings until the lock expires ---
            if now < pivot_lock_until:
                angular = -side * MAX_ANGULAR
                publish_cmd(client, 0.0, angular)
                if now - last_status >= STATUS_INTERVAL:
                    print(f"  🔒 Corner recovery — locked pivot away from "
                          f"{side_name} wall ({pivot_lock_until - now:.1f}s left)")
                    last_status = now
                continue

            # --- Inside corner: front blocked ---
            if front_d is not None and front_d < DANGER_DIST:
                if wall_lost:
                    # Combined case: outside corner approached obliquely,
                    # end-face now dead ahead with no side wall to follow.
                    # A single-step pivot would just be undone by the
                    # "wall lost" sweep next iteration — lock in a longer
                    # pivot-away to actually clear the corner.
                    pivot_lock_until = now + CORNER_RECOVERY_SEC
                    print(f"  🔒 Outside corner end-face detected "
                          f"(front={front_d:.2f}m, wall lost) — "
                          f"locking pivot away for {CORNER_RECOVERY_SEC:.1f}s")

                pivoting = True
                sweeping = False
                # Pivot AWAY from the followed wall (toward -side)
                angular = -side * MAX_ANGULAR
                publish_cmd(client, 0.0, angular)
                if now - last_status >= STATUS_INTERVAL:
                    print(f"  🔄 Inside corner — pivoting away from "
                          f"{side_name} wall (front={front_d:.2f}m)")
                    last_status = now
                continue

            if pivoting:
                # Front just cleared — resume normal following
                pivoting = False
                prev_dist_error = 0.0

            # --- Outside corner: wall lost ---
            if wall_lost:
                if not sweeping:
                    sweeping    = True
                    sweep_start = now
                    print(f"  ↪️  Wall lost (d_A="
                          f"{'None' if d_a is None else f'{d_a:.2f}m'}, "
                          f"d_B={'None' if d_b is None else f'{d_b:.2f}m'}"
                          f"{'' if d_est is None else f', d_est={d_est:.2f}m'}) "
                          f"— sweeping toward {side_name}")

                if now - sweep_start > SWEEP_TIMEOUT:
                    # Give up — drive straight, hope to find a wall ahead
                    publish_cmd(client, speed, 0.0)
                else:
                    # Turn toward the wall side while creeping forward
                    angular = side * (MAX_ANGULAR * 0.6)
                    publish_cmd(client, speed * 0.5, angular)
                continue
            else:
                if sweeping:
                    print(f"  ✅ Wall reacquired (d_est={d_est:.2f}m)")
                sweeping = False

            # --- Normal following: two-ray heading + distance control ---
            dist_error      = d_est - target_dist
            dist_d_error    = (dist_error - prev_dist_error) / dt
            prev_dist_error = dist_error

            # Heading-error term: positive when converging toward the
            # wall, negative when diverging (see geometric derivation).
            heading_term = d_a - d_b

            angular = side * (KP * dist_error + KD * dist_d_error
                              + KH * heading_term)
            angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

            publish_cmd(client, speed, angular)

            if now - last_status >= STATUS_INTERVAL:
                print(f"  ↔  d_est={d_est:.2f}m  err={dist_error:+.2f}m  "
                      f"d_A={d_a:.2f}m d_B={d_b:.2f}m "
                      f"heading_term={heading_term:+.2f}  "
                      f"front_d={'--' if front_d is None else f'{front_d:.2f}m'}  "
                      f"ω={angular:+.2f}")
                last_status = now

    except KeyboardInterrupt:
        print("\nWall-follower stopped.")
        publish_cmd(client, 0.0, 0.0)
        time.sleep(0.1)
    finally:
        publish_cmd(client, 0.0, 0.0)
        client.loop_stop()
        client.disconnect()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lidar-only wall-following exploration")
    parser.add_argument("--broker", default=DEFAULT_BROKER)
    parser.add_argument("--side", choices=["left", "right"], default="right",
                        help="Which wall to follow (default: right)")
    parser.add_argument("--target-dist", type=float, default=TARGET_DIST,
                        help=f"Standoff distance from wall in metres (default: {TARGET_DIST})")
    parser.add_argument("--speed", type=float, default=SPEED,
                        help=f"Forward speed in m/s (default: {SPEED})")
    args = parser.parse_args()

    run(args.broker, args.side, args.target_dist, args.speed)
