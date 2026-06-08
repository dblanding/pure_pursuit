#!/usr/bin/env python3
# obstacle_avoidance.py
"""
Obstacle avoidance node — supports two algorithms selectable via AVOIDANCE_MODE:

  "vfh"               Vector Field Histogram (Borenstein & Koren 1991)
                      Builds a polar obstacle density histogram, finds the
                      navigable valley closest to the goal direction, and
                      steers toward its centre.

  "potential_fields"  Artificial Potential Fields
                      Each obstacle exerts a repulsive force (∝ 1/d²).
                      The goal waypoint exerts an attractive force.
                      The robot steers in the direction of the net force.
                      Naturally centres in gaps without explicit valley finding.
                      Includes stuck detection + jiggle to escape local minima.

Sits between path_follower and the motor controller:

    path_follower  →  Topics.NAV_CMD_VEL  →  [this script]  →  Topics.MOTOR_CMD
                                                     ↑
                                          Topics.LIDAR_SCAN
                                          Topics.POSE
                                          Topics.NAV_GOAL

Three response states (both algorithms):

  CLEAR   No obstacles within influence range.
          path_follower commands passed through unchanged.

  STEER   Obstacles detected — algorithm overrides angular velocity.
          Linear velocity scaled down with proximity.

  STOP    Obstacle(s) within DANGER_DIST_M.
          Zero velocity until clear.

Startup order:
    1. icp_localizer.py
    2. obstacle_avoidance.py
    3. path_follower.py

Usage:
    python obstacle_avoidance.py
    python obstacle_avoidance.py --broker 192.168.1.85 --mode potential_fields
"""

import sys
import json
import math
import time
import threading
import argparse
import numpy as np
import paho.mqtt.client as mqtt

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Algorithm selection
# ---------------------------------------------------------------------------
# Set to "vfh" or "potential_fields"
AVOIDANCE_MODE = "vfh"

# ---------------------------------------------------------------------------
# Configuration — shared by both algorithms
# ---------------------------------------------------------------------------
DEFAULT_BROKER  = "raspibot.local"
BROKER_PORT     = 1883
MQTT_USERNAME   = "robot"
MQTT_PASSWORD   = "robot"

# Forward detection arc — only scan points within ±FORWARD_ARC_DEG/2
# of the robot's forward direction are considered.
FORWARD_ARC_DEG = 60.0          # ±30°

# Distance thresholds (metres)
DANGER_DIST_M   = 0.30          # inside this → STOP unconditionally
WARNING_DIST_M  = 0.50          # inside this → STEER (VFH) or always-on (APF)

# Maximum scan range to consider
MAX_SCAN_RANGE  = 3.0

# Minimum obstacle points to trigger a response (filters noise)
MIN_OBSTACLE_POINTS = 3   # warning zone: 3 points needed to trigger STEER
MIN_DANGER_POINTS   = 2   # danger zone: only 2 points needed to trigger STOP
                           # lower than MIN_OBSTACLE_POINTS to catch thin
                           # obstacles (chair legs) that produce sparse returns

# Maximum angular velocity either algorithm will command (rad/s)
MAX_ANGULAR     = 1.0

# --- Timing ---
CLEAR_HOLDOFF_SEC   = 0.5       # keep slowing after obstacle clears
CMD_WATCHDOG_SEC    = 1.0       # stop if path_follower goes silent
LOOP_RATE_HZ        = 20
CLEAR_CONFIRM_SCANS = 3         # consecutive clears needed to release STOP
STEER_MIN_HOLD_SEC  = 0.5       # min time in STEER before releasing

# Recovery rotation (VFH STOP+ROT mode)
# Once the robot commits to a rotation direction during recovery, it holds
# that direction for RECOVERY_ROT_MIN_SEC before re-evaluating.
# This prevents "chasing" a moving obstacle as the robot rotates around it.
RECOVERY_ROT_MIN_SEC   = 1.0    # seconds to hold locked rotation direction
RECOVERY_CLEAR_CONFIRM = 4      # consecutive clears needed after recovery

# Post-recovery corridor mode: after aligning with the gap, suppress VFH
# steering for this many seconds so the robot can drive straight through.
POST_RECOVERY_SEC = 3.0

# During corridor mode, only obstacles within this narrow arc abort the run.
# Side obstacles (gap walls) are ignored — alignment was already computed.
CORRIDOR_ARC_DEG  = 20.0   # ±10° — only head-on obstacles abort corridor

# ---------------------------------------------------------------------------
# VFH-specific configuration
# ---------------------------------------------------------------------------
N_HISTOGRAM_BINS   = 72         # 5° per bin
OBSTACLE_THRESHOLD = 3.0        # density above this = blocked
MIN_VALLEY_WIDTH   = 3          # bins minimum to count as a valley
K_VFH              = 3.0        # proportional gain: ω = K_VFH * valley_angle
WIDTH_WEIGHT       = 0.02       # width bonus in valley scoring (rad/bin)

# ---------------------------------------------------------------------------
# Potential fields-specific configuration
# ---------------------------------------------------------------------------
# Influence radius — scan points beyond this contribute no repulsion.
# Should be >= WARNING_DIST_M so APF activates before danger zone.
APF_INFLUENCE_RADIUS_M = 0.45

# Force gains — ratio determines behaviour:
#   K_REP >> K_ATT: robot stays well clear of obstacles, may not reach goal
#   K_ATT >> K_REP: robot ploughs toward goal, may get too close to obstacles
K_ATT = 1.0                     # attractive force toward goal
K_REP = 0.4                     # repulsive force from obstacles

# Proportional gain: ω = K_APF * steering_angle (from net force direction)
K_APF = 2.5

# Stuck detection — if the robot is in STEER mode but barely moving,
# it may be trapped in a local minimum.  Apply a brief sideways jiggle.
STUCK_TIME_SEC      = 2.0       # seconds before declaring stuck
STUCK_SPEED_THRESH  = 0.02      # m/s — below this = considered stuck
JIGGLE_DURATION_SEC = 0.4       # how long to jiggle (alternating turns)
JIGGLE_ANGULAR      = 0.8       # rad/s for jiggle turns

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
latest_scan = None
latest_pose = None
latest_cmd  = None
latest_goal = None
last_cmd_t  = 0.0
obstacle_t  = 0.0
lock        = threading.Lock()


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def scan_to_robot_frame(scan):
    """
    Convert lidar scan to list of (bearing_rad, distance_m).
    bearing: CCW positive, 0 = forward (+x).
    Filters to MAX_SCAN_RANGE.
    """
    pts = []
    for entry in scan:
        a = entry["a"]
        d = entry["d"]
        if 0.05 < d < MAX_SCAN_RANGE:
            pts.append((a, d))
    return pts


def goal_bearing_robot_frame(pose, goal):
    """
    Bearing to goal waypoint in robot frame, normalised to [-π, π].
    pose: {"x","y","h"}, goal: {"x","y"}
    """
    dx = goal["x"] - pose["x"]
    dy = goal["y"] - pose["y"]
    map_bearing   = math.atan2(dy, dx)
    robot_bearing = map_bearing - pose["h"]
    return math.atan2(math.sin(robot_bearing), math.cos(robot_bearing))


def count_forward_obstacles(pts):
    """Count danger/warning points in the forward arc."""
    half     = math.radians(FORWARD_ARC_DEG / 2.0)
    n_danger = 0
    n_warn   = 0
    closest  = float("inf")
    for bearing, dist in pts:
        b = math.atan2(math.sin(bearing), math.cos(bearing))
        if abs(b) > half:
            continue
        if dist < closest:
            closest = dist
        if dist < DANGER_DIST_M:
            n_danger += 1
        elif dist < WARNING_DIST_M:
            n_warn += 1
    return n_danger, n_warn, closest


def make_stop():
    return json.dumps({"linear": 0.0, "angular": 0.0,
                       "timestamp": time.time()})


# ===========================================================================
# VFH algorithm
# ===========================================================================

def _vfh_build_histogram(pts):
    hist     = np.zeros(N_HISTOGRAM_BINS)
    bin_size = 2 * math.pi / N_HISTOGRAM_BINS
    for bearing, dist in pts:
        b   = bearing % (2 * math.pi)
        idx = int(b / bin_size) % N_HISTOGRAM_BINS
        hist[idx] += 1.0 / (dist * dist)
    return hist


def _vfh_find_valleys(hist):
    n     = len(hist)
    free  = hist < OBSTACLE_THRESHOLD
    free2 = np.concatenate([free, free])
    valleys = []
    i = 0
    while i < n:
        if free2[i]:
            j = i
            while j < i + n and free2[j]:
                j += 1
            width = j - i
            if width >= MIN_VALLEY_WIDTH:
                centre_bin   = (i + width // 2) % n
                centre_angle = (centre_bin + 0.5) * (2 * math.pi / n)
                if centre_angle > math.pi:
                    centre_angle -= 2 * math.pi
                valleys.append((centre_angle, width))
            i = j
        else:
            i += 1
    return valleys


def _vfh_best_valley(valleys, preferred):
    if not valleys:
        return None
    def score(v):
        angle, width = v
        diff = abs(math.atan2(math.sin(angle - preferred),
                              math.cos(angle - preferred)))
        return diff - WIDTH_WEIGHT * width
    return min(valleys, key=score)


def _vfh_widest_valley(valleys):
    """Return the best recovery valley restricted to ±30° of forward.

    Using the full ±90° hemisphere lets wide sideways/rearward arcs dominate
    over narrow but correctly-positioned forward openings.  ±30° ensures
    recovery steers into the actual gap ahead rather than spinning into open
    floor space to the side.

    Falls back to nearest-to-forward if nothing is within ±30°.
    """
    if not valleys:
        return None

    max_rad = math.radians(30)
    forward = [v for v in valleys
               if abs(math.atan2(math.sin(v[0]), math.cos(v[0]))) <= max_rad]

    if not forward:
        # Nothing within ±30° — fall back to nearest-to-forward from full set
        return min(valleys,
                   key=lambda v: abs(math.atan2(math.sin(v[0]),
                                                math.cos(v[0]))))

    # Among forward valleys pick the widest
    best = max(forward, key=lambda v: v[1])

    # Debug line (remove after tuning is complete)
    all_v  = [(round(math.degrees(v[0]), 1), v[1]) for v in valleys]
    fwd_v  = [(round(math.degrees(v[0]), 1), v[1]) for v in forward]
    print(f"  [recovery] all={all_v}  fwd±30°={fwd_v}  "
          f"chose={round(math.degrees(best[0]), 1)}° w={best[1]}")
    return best


def analyse_vfh(scan, pose, goal):
    """
    VFH analysis. Returns result dict with keys:
        mode, scale, angular_override, closest, detail,
        recovery_angular  — in-place rotation toward best valley during STOP
                            (None if no valley found)
    """
    pts               = scan_to_robot_frame(scan)
    n_danger, n_warn, closest = count_forward_obstacles(pts)

    # Always build histogram so recovery_angular is available even during STOP
    hist      = _vfh_build_histogram(pts)
    valleys   = _vfh_find_valleys(hist)
    preferred = goal_bearing_robot_frame(pose, goal) \
                if (pose and goal) else 0.0
    chosen    = _vfh_best_valley(valleys, preferred)

    # Recovery angular: rotate toward the WIDEST valley (most open space),
    # ignoring goal direction.  This ensures recovery spins AWAY from the
    # obstacle even when the goal happens to be on the obstacle's side.
    recovery_angular = None
    widest = _vfh_widest_valley(valleys)
    if widest is not None:
        va_r = widest[0]
        recovery_angular = max(-MAX_ANGULAR,
                               min(MAX_ANGULAR, K_VFH * va_r))

    if n_danger >= MIN_DANGER_POINTS:
        return dict(mode="stop", scale=0.0, angular_override=None,
                    closest=closest, detail=f"danger={n_danger}pts",
                    recovery_angular=recovery_angular)

    if n_warn < MIN_OBSTACLE_POINTS:
        return dict(mode="clear", scale=1.0, angular_override=None,
                    closest=closest, detail="",
                    recovery_angular=recovery_angular)

    if chosen is None:
        return dict(mode="stop", scale=0.0, angular_override=None,
                    closest=closest, detail="no valley",
                    recovery_angular=None)

    scale   = max(0.1, (closest - DANGER_DIST_M) / (WARNING_DIST_M - DANGER_DIST_M))
    va, vw  = chosen
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_VFH * va))
    detail  = (f"valley={math.degrees(va):.1f}° w={vw}bins "
               f"goal={math.degrees(preferred):.1f}°")
    return dict(mode="steer", scale=scale, angular_override=angular,
                closest=closest, detail=detail,
                recovery_angular=recovery_angular)


# ===========================================================================
# Potential fields algorithm
# ===========================================================================

def analyse_apf(scan, pose, goal):
    """
    Artificial Potential Fields analysis.

    Repulsive force from each scan point within APF_INFLUENCE_RADIUS_M,
    scaled by (1/d - 1/d_inf)² where d_inf = APF_INFLUENCE_RADIUS_M.
    Attractive force toward goal waypoint.
    Robot steers in direction of net force vector.

    Returns result dict with same keys as analyse_vfh.
    """
    pts               = scan_to_robot_frame(scan)
    n_danger, n_warn, closest = count_forward_obstacles(pts)

    # Always stop if danger zone breached
    if n_danger >= MIN_DANGER_POINTS:
        return dict(mode="stop", scale=0.0, angular_override=None,
                    closest=closest, detail=f"danger={n_danger}pts")

    # --- Compute repulsive force vector (in robot frame) ---
    F_rep_x = 0.0
    F_rep_y = 0.0
    half    = math.radians(FORWARD_ARC_DEG / 2.0)
    d_inf   = APF_INFLUENCE_RADIUS_M

    for bearing, dist in pts:
        b = math.atan2(math.sin(bearing), math.cos(bearing))
        if abs(b) > half:
            continue                    # outside forward arc
        if dist >= d_inf:
            continue                    # beyond influence radius

        # APF repulsive force magnitude: (1/d - 1/d_inf)² / d²
        # Direction: away from obstacle (opposite to bearing)
        magnitude = K_REP * (1.0/dist - 1.0/d_inf)**2 / (dist**2)
        # Unit vector AWAY from obstacle
        F_rep_x += magnitude * (-math.cos(b))
        F_rep_y += magnitude * (-math.sin(b))

    # --- Compute attractive force toward goal (in robot frame) ---
    F_att_x = 0.0
    F_att_y = 0.0
    if pose is not None and goal is not None:
        goal_bearing = goal_bearing_robot_frame(pose, goal)
        goal_dist    = math.hypot(goal["x"] - pose["x"],
                                  goal["y"] - pose["y"])
        # Attractive force: K_ATT * unit vector toward goal
        F_att_x = K_ATT * math.cos(goal_bearing)
        F_att_y = K_ATT * math.sin(goal_bearing)

    # --- Net force ---
    F_x = F_att_x + F_rep_x
    F_y = F_att_y + F_rep_y
    F_mag = math.hypot(F_x, F_y)

    # Steering angle = direction of net force relative to robot forward
    if F_mag < 1e-6:
        steer_angle = 0.0       # forces cancel — go straight
    else:
        steer_angle = math.atan2(F_y, F_x)

    # For steering, use only the LATERAL (y) component of net force.
    # The full 2D force direction is misleading for side obstacles because
    # the repulsion vector points backward-and-sideways (away from the obstacle),
    # causing a false "blocked" reading. The lateral imbalance is what steers
    # the robot; the forward component determines speed scaling.
    #
    # Steering angle: atan2(F_lateral, |F_forward|) gives a meaningful
    # left/right signal regardless of obstacle angle.
    F_lateral  = F_y                    # positive = steer left
    F_forward  = F_x                    # positive = net force has forward component

    # Repulsion negligible → treat as clear
    rep_mag = math.hypot(F_rep_x, F_rep_y)
    if rep_mag < 0.05:
        return dict(mode="clear", scale=1.0, angular_override=None,
                    closest=closest, detail="")

    # Head-on stop: only if danger-zone points present AND net force backward
    # (avoids stopping in a corridor where repulsion is symmetric but manageable)
    if n_danger >= MIN_OBSTACLE_POINTS and F_forward < -0.5 and abs(F_lateral) < 0.1:
        return dict(mode="stop", scale=0.0, angular_override=None,
                    closest=closest,
                    detail=f"head-on obstacle (Fx={F_forward:.2f})")

    # Steer proportional to lateral force imbalance
    steer_angle = math.atan2(F_lateral, max(0.1, F_forward))

    # Scale linear velocity by proximity within the influence radius.
    # Robot always slows when APF is active — full speed at influence boundary,
    # minimum speed at danger zone.
    scale = max(0.15, (closest - DANGER_DIST_M) /
                      (APF_INFLUENCE_RADIUS_M - DANGER_DIST_M))

    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, K_APF * steer_angle))
    detail  = (f"F_lat={F_lateral:.2f} F_fwd={F_forward:.2f} "
               f"steer={math.degrees(steer_angle):.1f}°")

    return dict(mode="steer", scale=scale, angular_override=angular,
                closest=closest, detail=detail)


# ===========================================================================
# Algorithm dispatcher
# ===========================================================================

def analyse(scan, pose, goal, mode=AVOIDANCE_MODE):
    if mode == "potential_fields":
        return analyse_apf(scan, pose, goal)
    return analyse_vfh(scan, pose, goal)


# ---------------------------------------------------------------------------
# MQTT
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.LIDAR_SCAN)
        client.subscribe(Topics.NAV_CMD_VEL)
        client.subscribe(Topics.POSE)
        client.subscribe(Topics.NAV_GOAL)
        algo = "Potential Fields (APF)" if AVOIDANCE_MODE == "potential_fields" \
               else "Vector Field Histogram (VFH)"
        print(f"Connected. Algorithm: {algo}")
        print(f"  Listening:  {Topics.LIDAR_SCAN}  {Topics.NAV_CMD_VEL}")
        print(f"              {Topics.POSE}  {Topics.NAV_GOAL}")
        print(f"  Forwarding: {Topics.MOTOR_CMD}")
        print(f"  Danger: {DANGER_DIST_M:.2f}m  "
              f"Influence: {APF_INFLUENCE_RADIUS_M:.2f}m  "
              f"Arc: ±{FORWARD_ARC_DEG/2:.0f}°")
    else:
        print(f"Connection failed rc={rc}")


def on_message(client, userdata, msg):
    global latest_scan, latest_pose, latest_cmd, latest_goal, last_cmd_t
    try:
        payload = json.loads(msg.payload.decode())
        with lock:
            if msg.topic == Topics.LIDAR_SCAN:
                latest_scan = payload
            elif msg.topic == Topics.NAV_CMD_VEL:
                latest_cmd = payload
                last_cmd_t = time.monotonic()
            elif msg.topic == Topics.POSE:
                latest_pose = payload
            elif msg.topic == Topics.NAV_GOAL:
                latest_goal = payload
    except Exception as e:
        print(f"Message error on {msg.topic}: {e}")


def _narrow_arc_danger(scan):
    """Return True if DANGER_DIST_M is breached within the narrow corridor arc.
    Used during post-recovery to ignore side obstacles and only stop for
    something genuinely blocking the forward path."""
    if scan is None:
        return False
    half = math.radians(CORRIDOR_ARC_DEG / 2.0)
    count = 0
    for entry in scan:
        a = entry["a"]
        d = entry["d"]
        b = math.atan2(math.sin(a), math.cos(a))
        if abs(b) <= half and 0.05 < d < DANGER_DIST_M:
            count += 1
            if count >= MIN_DANGER_POINTS:
                return True
    return False


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def run(broker, mode):
    global obstacle_t

    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, BROKER_PORT, 60)
    client.loop_start()

    print(f"Obstacle avoidance starting ({mode}) — connecting to {broker}...")
    prev_mode         = "clear"
    loop_dt           = 1.0 / LOOP_RATE_HZ
    consecutive_clear = 0
    steer_started_t   = 0.0

    # Stuck detection state (APF only)
    steer_enter_t     = 0.0        # when we entered STEER mode
    jiggle_until_t    = 0.0        # jiggle active until this time
    jiggle_sign       = 1          # alternating jiggle direction

    # Recovery rotation state (VFH STOP+ROT)
    recovery_dir      = 0.0        # locked rotation direction (±1.0)
    recovery_lock_t   = 0.0        # time direction was locked
    in_recovery       = False      # True while rotating out of a stop
    post_recovery_until_t = 0.0   # suppress STEER until this time after recovery

    try:
        while True:
            time.sleep(loop_dt)
            now = time.monotonic()

            with lock:
                scan    = latest_scan
                pose    = latest_pose
                cmd     = latest_cmd
                goal    = latest_goal
                cmd_age = now - last_cmd_t

            # Watchdog
            if cmd is not None and cmd_age > CMD_WATCHDOG_SEC:
                client.publish(Topics.MOTOR_CMD, make_stop())
                if prev_mode != "watchdog":
                    print(f"⚠️  Watchdog: no command for {cmd_age:.1f}s")
                prev_mode = "watchdog"
                continue

            if scan is None or cmd is None:
                continue

            result = analyse(scan, pose, goal, mode)
            rmode  = result["mode"]

            # Consecutive clear tracking
            if rmode == "clear":
                consecutive_clear += 1
            else:
                consecutive_clear = 0
            # Use stricter clear confirmation after recovery rotation
            # to ensure robot is properly aligned before resuming
            needed_clears   = (RECOVERY_CLEAR_CONFIRM if in_recovery
                               else CLEAR_CONFIRM_SCANS)
            confirmed_clear = (rmode == "clear" and
                               consecutive_clear >= needed_clears)

            # Holdoffs
            if rmode != "clear":
                obstacle_t = now
            in_holdoff    = (now - obstacle_t)     < CLEAR_HOLDOFF_SEC
            in_steer_hold = (now - steer_started_t) < STEER_MIN_HOLD_SEC

            # Post-recovery corridor mode: ignore STEER, honour only STOP
            # within a narrow forward arc (±CORRIDOR_ARC_DEG/2) so side
            # obstacles don't abort the corridor run.
            in_post_recovery = (now < post_recovery_until_t)
            corridor_stop    = (in_post_recovery and
                                result["mode"] == "stop" and
                                _narrow_arc_danger(scan))

            # Track how long we've been in STEER
            if rmode == "steer" and prev_mode != "steer":
                steer_enter_t = now
                steer_started_t = now

            # --- STOP (honoured always, but during corridor only for head-on danger) ---
            if (not in_post_recovery and (rmode == "stop" or (in_holdoff and prev_mode == "stop"))) \
               or corridor_stop:
                rec = result.get("recovery_angular")
                if mode == "vfh" and rec is not None:
                    # Direction-locked recovery rotation.
                    # On entering recovery, lock the direction for
                    # RECOVERY_ROT_MIN_SEC.  This prevents the robot from
                    # chasing an obstacle as it sweeps across the field of
                    # view during rotation.
                    if not in_recovery:
                        # Entering recovery — lock direction now
                        recovery_dir   = math.copysign(1.0, rec)
                        recovery_lock_t = now
                        in_recovery    = True
                        print(f"🔄 STOP+ROT  closest={result['closest']:.3f}m  "
                              f"locking rotation ω={recovery_dir*MAX_ANGULAR:+.2f}  "
                              f"{result['detail']}")
                    elif (now - recovery_lock_t) > RECOVERY_ROT_MIN_SEC:
                        # Lock expired — re-evaluate direction
                        new_dir = math.copysign(1.0, rec)
                        if new_dir != recovery_dir:
                            recovery_dir    = new_dir
                            recovery_lock_t = now
                            print(f"🔄 STOP+ROT  direction updated "
                                  f"ω={recovery_dir*MAX_ANGULAR:+.2f}")

                    out = {"linear": 0.0,
                           "angular": round(recovery_dir * MAX_ANGULAR, 4),
                           "timestamp": time.time()}
                    client.publish(Topics.MOTOR_CMD, json.dumps(out))
                else:
                    in_recovery = False
                    client.publish(Topics.MOTOR_CMD, make_stop())
                    if prev_mode != "stop":
                        print(f"🛑 STOP  closest={result['closest']:.3f}m  "
                              f"{result['detail']}")
                prev_mode = "stop"
                steer_enter_t = 0.0

            # --- POST-RECOVERY CORRIDOR (STEER suppressed) ---
            elif in_post_recovery:
                # Robot just aligned with the gap via recovery rotation.
                # Drive straight on the current heading — use path_follower's
                # linear speed but force angular=0 so we don't immediately
                # turn back toward the waypoint and drift out of the gap.
                out = {"linear":    round(cmd["linear"], 4),
                       "angular":   0.0,
                       "timestamp": time.time()}
                client.publish(Topics.MOTOR_CMD, json.dumps(out))
                if prev_mode != "post_recovery":
                    pass   # entry message already printed
                prev_mode = "post_recovery"

            # --- STEER ---
            elif rmode == "steer" or in_holdoff or in_steer_hold:

                # Stuck detection (APF — VFH doesn't need it as much)
                if (mode == "potential_fields" and
                        rmode == "steer" and
                        (now - steer_enter_t) > STUCK_TIME_SEC and
                        abs(cmd.get("linear", 0)) < STUCK_SPEED_THRESH):
                    # Trigger a jiggle
                    jiggle_until_t = now + JIGGLE_DURATION_SEC
                    jiggle_sign    = -jiggle_sign
                    steer_enter_t  = now   # reset timer
                    print(f"🔄 Jiggle (stuck in local minimum)")

                if now < jiggle_until_t:
                    # Jiggle: spin in place alternating directions
                    out = {"linear": 0.0,
                           "angular": jiggle_sign * JIGGLE_ANGULAR,
                           "timestamp": time.time()}
                    client.publish(Topics.MOTOR_CMD, json.dumps(out))
                else:
                    out = dict(cmd)
                    out["linear"]    = round(cmd["linear"] * result["scale"], 4)
                    out["angular"]   = round(
                        result["angular_override"] if result["angular_override"] is not None
                        else cmd["angular"], 4)
                    out["timestamp"] = time.time()
                    client.publish(Topics.MOTOR_CMD, json.dumps(out))

                if prev_mode != "steer":
                    print(f"↗  STEER  closest={result['closest']:.3f}m  "
                          f"scale={result['scale']:.2f}  "
                          f"ω={result['angular_override'] or 0:+.2f}  "
                          f"{result['detail']}")
                prev_mode = "steer"

            # --- CLEAR ---
            elif confirmed_clear:
                out = dict(cmd)
                out["timestamp"] = time.time()
                client.publish(Topics.MOTOR_CMD, json.dumps(out))
                if prev_mode != "clear":
                    print(f"✅ Clear  closest={result['closest']:.3f}m"
                          + (" (after recovery)" if in_recovery else ""))
                if in_recovery:
                    post_recovery_until_t = now + POST_RECOVERY_SEC
                    print(f"🟢 POST-RECOVERY corridor mode for {POST_RECOVERY_SEC:.1f}s "
                          f"— VFH STEER suppressed, danger-only stops")
                in_recovery   = False   # recovery complete
                recovery_dir  = 0.0
                prev_mode     = "clear"
                steer_enter_t = 0.0

            # --- UNCONFIRMED CLEAR ---
            else:
                if prev_mode == "stop":
                    client.publish(Topics.MOTOR_CMD, make_stop())
                else:
                    out = dict(cmd)
                    out["linear"]    = round(cmd["linear"] * 0.5, 4)
                    out["timestamp"] = time.time()
                    client.publish(Topics.MOTOR_CMD, json.dumps(out))

    except KeyboardInterrupt:
        print(f"\n{mode} obstacle avoidance stopped.")
        client.publish(Topics.MOTOR_CMD, make_stop())
    finally:
        client.loop_stop()
        client.disconnect()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Obstacle avoidance node")
    parser.add_argument("--broker", default=DEFAULT_BROKER,
                        help=f"MQTT broker (default: {DEFAULT_BROKER})")
    parser.add_argument("--mode", default=AVOIDANCE_MODE,
                        choices=["vfh", "potential_fields"],
                        help="Avoidance algorithm (default: from AVOIDANCE_MODE)")
    args = parser.parse_args()
    run(args.broker, args.mode)
