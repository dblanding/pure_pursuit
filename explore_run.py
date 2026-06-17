#!/usr/bin/env python3
# explore_run.py
"""
Drive from home (0, 0) to a waypoint using pure-pursuit, then hand off to
left-wall-following for perimeter exploration.

Phase 1 — pure pursuit to waypoint:
  - Subscribes to Topics.POSE (from icp_localizer.py) for position.
  - Publishes to Topics.NAV_CMD_VEL so obstacle_avoidance.py stays in the
    safety loop during the drive.

Phase 2 — wall-following:
  - Pure reactive lidar loop (no pose needed).
  - Publishes directly to Topics.MOTOR_CMD, bypassing obstacle_avoidance.py
    (wall-following deliberately runs close to walls, inside VFH's warning
    zone, and needs its own simpler front-danger check instead).

Usage:
    uv run explore_run.py
    uv run explore_run.py --waypoint -1.05 2.90 --side left
    uv run explore_run.py --waypoint -1.05 2.90 --side left --speed 0.18

Terminals needed:
    1. uv run icp_localizer.py
    2. uv run obstacle_avoidance.py      (Phase 1 safety; can stop after handoff)
    3. uv run explore_run.py
"""

import sys
import json
import math
import time
import argparse
import threading
from collections import deque

import paho.mqtt.client as mqtt
import numpy as np

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Shared configuration
# ---------------------------------------------------------------------------
BROKER          = "raspibot.local"
BROKER_PORT     = 1883
MQTT_USERNAME   = "robot"
MQTT_PASSWORD   = "robot"

# Phase 1 — pure-pursuit
PP_MAX_LINEAR   = 0.20    # m/s — drive speed to waypoint
PP_MAX_ANGULAR  = 1.0     # rad/s
PP_K_P          = 2.0     # heading proportional gain
PP_D_GAIN       = 0.5     # heading derivative damping
PP_LOOKAHEAD    = 0.30    # m — lookahead distance
GOAL_TOLERANCE  = 0.15    # m — close-enough to waypoint

# Phase 2 — wall following
WF_SPEED        = 0.18    # m/s forward speed while following
WF_TARGET_DIST  = 0.40    # m — desired standoff from wall
WF_MAX_ANGULAR  = 1.0     # rad/s

ANGLE_A         = 70.0    # degrees — front-leaning ray (early corner detection)
ANGLE_B         = 110.0   # degrees — rear-leaning ray (heading error paired with A)
ANGLE_C         = 90.0    # degrees — directly abeam; sin(90°)=1 so d_C = exact
                           # perpendicular distance; last to lose wall at corner
RAY_WINDOW_DEG  = 10.0    # ± window for each ray sample
SIN_ANGLE_A     = math.sin(math.radians(ANGLE_A))  # for single-ray fallback only

FRONT_ARC_DEG   = 30.0    # ± half-width of front danger arc
                           # Narrower than the old 40° to avoid false
                           # inside-corner pivots from diagonal bay-window
                           # walls that clip the edge of the front arc
DANGER_DIST     = 0.25    # m — triggers inside-corner pivot
                           # Reduced from 0.30m; angled walls close to
                           # the side can read ~0.28-0.30m in the forward
                           # arc without being a genuine collision risk
SIDE_DANGER_DIST = 0.15   # m — if either side ray drops below this,
                           # the robot's body is about to contact the wall;
                           # pivot away immediately regardless of front arc
LOST_WALL_DIST  = 0.80    # m — d_est above this = wall lost

KP              = 2.5     # distance PD gains
KD              = 1.5
KH              = 3.0     # heading-error gain (two-ray term)

SMOOTH_WINDOW    = 4
NONE_RESET_COUNT = 2

CORNER_ARC_COOLDOWN    = 0.5    # s after arc completes before next can fire.
POST_ARC_STRAIGHT_SEC  = 2.0    # s after arc: drive straight, no sweep.
                                # After a 90° arc the new wall converges into
                                # view naturally as the robot moves forward;
                                # sweeping only adds unwanted extra rotation.
ARC_TRIGGER_WINDOW     = 1.0    # s — wall_lost arc only fires if d_est was
                                # below LOST_WALL_DIST within this window.
                                # Prevents cascading fallback arcs when the
                                # robot is genuinely lost (no wall seen for
                                # several seconds after multiple arcs).
                                # Previously 4s then 2s, but the ratio check
                                # (RAY_RATIO_THRESHOLD) eliminated the cascade
                                # that needed a long cooldown.  Now just long
                                # enough to prevent same-scan re-triggers while
                                # still allowing consecutive corners to be detected
                                # almost immediately (0.5s = 0.09m travel).
CORNER_ARC_MIN_RADIUS  = 0.25   # m — don't fire arc if d_perp is this small
POST_ARC_STRAIGHT_SEC  = 1.5    # s — after arc, drive straight before sweeping;
                                # gives the new wall time to appear in both rays
                                # naturally rather than having the sweep push the
                                # robot too close before it can stabilise
                                # that reading is likely a close obstacle handled
                                # by side-danger, not a valid corner radius

SWEEP_TIMEOUT        = 5.0    # s — give up outside-corner sweep, go straight
SWEEP_ANGULAR_RATE   = 0.25   # fraction of WF_MAX_ANGULAR to use while sweeping
CORNER_RECOVERY_SEC  = 1.5    # s — locked pivot-away at oblique outside corner
CORNER_REVERSE_SEC   = 0.4    # s — back up before pivoting at outside corner,
                               # to clear the rear corner from the end-face
                               # before the pivot arc sweeps it into the wall

STATUS_INTERVAL = 5.0         # s between status prints

MIN_RANGE = 0.05
MAX_RANGE = 4.0

# ---------------------------------------------------------------------------
# Shared MQTT state
# ---------------------------------------------------------------------------
_lock         = threading.Lock()
_pose         = None          # {"x":, "y":, "h":, "hr":}
_pose_updated = False
_latest_scan  = None
_new_scan     = False


def _on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.POSE)
        client.subscribe(Topics.LIDAR_SCAN)
    else:
        print(f"MQTT connection failed rc={rc}")


def _on_message(client, userdata, msg):
    global _pose, _pose_updated, _latest_scan, _new_scan
    try:
        data = json.loads(msg.payload.decode())
        with _lock:
            if msg.topic == Topics.POSE:
                _pose = data
                _pose_updated = True
            elif msg.topic == Topics.LIDAR_SCAN:
                _latest_scan = data
                _new_scan = True
    except Exception as e:
        print(f"Parse error: {e}")


def publish_nav(client, linear, angular):
    """Phase 1: publish to NAV_CMD_VEL (through obstacle_avoidance)."""
    cmd = {"linear": round(float(linear), 4),
           "angular": round(float(angular), 4),
           "timestamp": time.time()}
    client.publish(Topics.NAV_CMD_VEL, json.dumps(cmd))


def publish_motor(client, linear, angular):
    """Phase 2: publish directly to MOTOR_CMD (bypass obstacle_avoidance)."""
    cmd = {"linear": round(float(linear), 4),
           "angular": round(float(angular), 4),
           "timestamp": time.time()}
    client.publish(Topics.MOTOR_CMD, json.dumps(cmd))


# ---------------------------------------------------------------------------
# Phase 1 — pure pursuit to waypoint
# ---------------------------------------------------------------------------

def wait_for_pose(timeout=10.0):
    global _pose_updated
    print("⏳ Waiting for pose from icp_localizer...")
    t0 = time.monotonic()
    while not _pose_updated:
        if time.monotonic() - t0 > timeout:
            return False
        time.sleep(0.1)
    with _lock:
        p = _pose
    print(f"✅ Pose received: ({p['x']:.3f}, {p['y']:.3f}, "
          f"{math.degrees(p['h']):.1f}°)")
    return True


def drive_to_waypoint(client, wx, wy):
    """
    Pure-pursuit drive from current pose to (wx, wy).
    Returns True on success, False if interrupted.
    """
    print(f"\n   ↗  Driving to ({wx:.3f}, {wy:.3f}) ..."
          f"  (publishing to {Topics.NAV_CMD_VEL})")

    prev_angle_error = 0.0
    prev_t      = time.monotonic()
    last_status = time.monotonic()

    while True:
        time.sleep(0.05)
        now = time.monotonic()
        dt = max(now - prev_t, 1e-3)
        prev_t = now

        with _lock:
            p = _pose
        if p is None:
            continue

        rx, ry, rtheta = p['x'], p['y'], p['h']
        romega = p.get('hr', 0.0)

        dx = wx - rx
        dy = wy - ry
        dist = math.hypot(dx, dy)

        if now - last_status >= STATUS_INTERVAL:
            print(f"  ↗  pos=({rx:.2f}, {ry:.2f})  "
                  f"dist_to_waypoint={dist:.2f}m  "
                  f"heading={math.degrees(rtheta):.1f}°")
            last_status = now

        if dist < GOAL_TOLERANCE:
            print(f"  ✅ Waypoint reached  (dist={dist:.2f}m)")
            publish_nav(client, 0.0, 0.0)
            return True

        # Proportional + derivative heading control
        target_angle = math.atan2(dy, dx)
        angle_error  = math.atan2(math.sin(target_angle - rtheta),
                                   math.cos(target_angle - rtheta))

        if dist > PP_LOOKAHEAD:
            linear = PP_MAX_LINEAR
        else:
            linear = PP_MAX_LINEAR * (dist / PP_LOOKAHEAD)

        angular = PP_K_P * angle_error - PP_D_GAIN * romega
        angular = max(-PP_MAX_ANGULAR, min(PP_MAX_ANGULAR, angular))

        publish_nav(client, linear, angular)


# ---------------------------------------------------------------------------
# Phase 2 — wall following helpers
# ---------------------------------------------------------------------------

def _norm_deg(a):
    a = a % 360.0
    return a - 360.0 if a > 180.0 else a


def distance_at_angle(scan, target_deg, window_deg=RAY_WINDOW_DEG):
    best = None
    for entry in scan:
        a_deg = _norm_deg(math.degrees(entry["a"]))
        d = entry["d"]
        if d < MIN_RANGE or d > MAX_RANGE:
            continue
        if abs(_norm_deg(a_deg - target_deg)) <= window_deg:
            if best is None or d < best:
                best = d
    return best


def front_min_distance(scan):
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


def rotate_to_heading(client, target_deg, tolerance_deg=5.0):
    """
    Rotate in place to face target_deg (degrees, math convention:
    0=east/+x, 90=north/+y, 180=west/-x, -90=south/-y).
    Uses NAV_CMD_VEL so obstacle_avoidance stays in the safety loop.
    """
    print(f"\n🔄 Aligning to heading {target_deg:.0f}° ...")
    target_rad = math.radians(target_deg)

    while True:
        time.sleep(0.05)
        with _lock:
            p = _pose
        if p is None:
            continue

        heading = p['h']   # current heading in radians
        error = math.atan2(math.sin(target_rad - heading),
                           math.cos(target_rad - heading))

        if abs(math.degrees(error)) < tolerance_deg:
            publish_nav(client, 0.0, 0.0)
            print(f"  ✅ Aligned  (heading={math.degrees(heading):.1f}°, "
                  f"error={math.degrees(error):+.1f}°)")
            return

        angular = max(-1.0, min(1.0, 2.5 * error))
        publish_nav(client, 0.0, angular)



# ---------------------------------------------------------------------------
# Phase 2 — wall-following loop
# ---------------------------------------------------------------------------

def follow_walls(client, side_name):
    """
    Reactive left/right wall-following using two-ray heading+distance PD control.
    Publishes directly to MOTOR_CMD.
    """
    global _new_scan

    side = +1 if side_name == "left" else -1
    print(f"\n🧭 Phase 2 — wall following ({side_name} wall, "
          f"target={WF_TARGET_DIST}m, speed={WF_SPEED}m/s)")
    print(f"   Publishing to {Topics.MOTOR_CMD} (direct, bypassing obstacle_avoidance)")

    prev_dist_error   = 0.0
    prev_t            = time.monotonic()
    last_status       = time.monotonic()

    sweeping          = False
    sweep_start       = 0.0

    pivoting          = False
    pivot_lock_until  = 0.0
    reverse_until     = 0.0   # legacy, no longer used actively

    # Geometric outside-corner arc state.
    # Fires when d_A (front ray) loses the wall while d_B (rear ray) still
    # sees it — meaning the robot's front has passed the corner.  Executes
    # a 90° arc with radius = current wall distance, which is exactly the
    # maneuver needed to round the corner and arrive parallel to the new wall.
    corner_arc_until   = 0.0
    corner_arc_angular = 0.0   # computed per-corner from actual wall distance
    corner_arc_cooldown_until = 0.0   # no new arc until this time
    post_arc_straight_until   = 0.0   # drive straight (no sweep) until this time
    was_corner_arcing  = False        # tracks arc → completion transition
    last_close_wall_t  = 0.0         # last time d_est was below LOST_WALL_DIST

    d_a_buf           = deque(maxlen=SMOOTH_WINDOW)
    d_b_buf           = deque(maxlen=SMOOTH_WINDOW)
    d_c_buf           = deque(maxlen=SMOOTH_WINDOW)   # 90° abeam ray
    consec_none_a     = 0
    consec_none_b     = 0
    consec_none_c     = 0

    while True:
        time.sleep(0.05)
        now = time.monotonic()

        with _lock:
            scan  = _latest_scan
            ready = _new_scan
            if ready:
                _new_scan = False

        if scan is None or not ready:
            continue

        dt = max(now - prev_t, 1e-3)
        prev_t = now

        # --- Geometric corner arc: HIGHEST PRIORITY ---
        # Executes a 90° arc (radius = wall distance) to round an outside
        # corner.  Must be checked before all other logic so nothing
        # interrupts it once started.
        if now < corner_arc_until:
            was_corner_arcing = True
            publish_motor(client, WF_SPEED, side * corner_arc_angular)
            if now - last_status >= STATUS_INTERVAL:
                print(f"  🔄 Corner arc ω={side*corner_arc_angular:+.2f} "
                      f"({corner_arc_until - now:.1f}s left)")
                last_status = now
            continue

        if was_corner_arcing:
            # Arc just finished — start cooldown before allowing another arc,
            # and drive straight briefly so the new wall converges into view
            # without the sweep adding extra rotation.
            was_corner_arcing = False
            corner_arc_cooldown_until = now + CORNER_ARC_COOLDOWN
            post_arc_straight_until   = now + POST_ARC_STRAIGHT_SEC
            print(f"  ✓ Corner arc complete — straight drive {POST_ARC_STRAIGHT_SEC:.0f}s "
                  f"then PD (cooldown {CORNER_ARC_COOLDOWN:.1f}s)")

        # --- Corner-recovery arc: HIGHEST PRIORITY ---
        # Must be checked before ANYTHING else (side-danger, front-danger,
        # wall-lost) so the recovery sequence runs to completion without
        # being interrupted by sensor readings that are inherently noisy
        # near a corner.
        #
        # Strategy: drive forward at half speed while turning hard away from
        # the wall side.  This swings the nose clear of the end-face in the
        # same motion, rather than reversing (which keeps the nose pointing
        # at the obstacle) or pivoting in place (which sweeps the rear into
        # the end-face).
        if now < pivot_lock_until:
            publish_motor(client, WF_SPEED * 0.5, -side * WF_MAX_ANGULAR)
            if now - last_status >= STATUS_INTERVAL:
                print(f"  🔒 Corner recovery arc "
                      f"({pivot_lock_until - now:.1f}s left)")
                last_status = now
            continue

        front_d = front_min_distance(scan)

        d_a_raw = distance_at_angle(scan, side * ANGLE_A)
        d_b_raw = distance_at_angle(scan, side * ANGLE_B)
        d_c_raw = distance_at_angle(scan, side * ANGLE_C)   # 90° abeam

        # Smooth each ray independently
        if d_a_raw is not None:
            d_a_buf.append(d_a_raw);  consec_none_a = 0
        else:
            consec_none_a += 1
            if consec_none_a >= NONE_RESET_COUNT:
                d_a_buf.clear()

        if d_b_raw is not None:
            d_b_buf.append(d_b_raw);  consec_none_b = 0
        else:
            consec_none_b += 1
            if consec_none_b >= NONE_RESET_COUNT:
                d_b_buf.clear()

        if d_c_raw is not None:
            d_c_buf.append(d_c_raw);  consec_none_c = 0
        else:
            consec_none_c += 1
            if consec_none_c >= NONE_RESET_COUNT:
                d_c_buf.clear()

        d_a = (sum(d_a_buf) / len(d_a_buf)) if d_a_buf else None
        d_b = (sum(d_b_buf) / len(d_b_buf)) if d_b_buf else None
        d_c = (sum(d_c_buf) / len(d_c_buf)) if d_c_buf else None

        # --- Side-danger: obstacle too close on the followed side ---
        # The two rays distinguish WHERE the contact is:
        #
        #   d_A (front-leaning, 70°) close → obstacle at front-left corner
        #     → pivot AWAY (existing behaviour) — turning pulls the front clear
        #
        #   d_B (rear-leaning, 110°) close but d_A clear → obstacle catching
        #     the REAR-left corner (e.g. peninsula end-face as robot pivots)
        #     → drive FORWARD — this pulls the rear corner away from the
        #     obstacle; reversing or pivoting would push it harder in.
        #
        #   Both close → wall is too close along its full length → pivot away.

        front_side_close = d_a is not None and d_a < SIDE_DANGER_DIST
        rear_side_close  = d_b is not None and d_b < SIDE_DANGER_DIST

        if front_side_close or (front_side_close and rear_side_close):
            # Front contact (or both) — pivot away
            sweeping = False
            pivoting = True
            publish_motor(client, 0.0, -side * WF_MAX_ANGULAR)
            if now - last_status >= STATUS_INTERVAL:
                print(f"  ⚠️  Front-side danger — pivoting away "
                      f"(d_A={d_a:.2f}m, "
                      f"d_B={'--' if d_b is None else f'{d_b:.2f}m'})")
                last_status = now
            continue

        if rear_side_close and not front_side_close:
            # Rear corner caught — drive forward to pull it clear
            sweeping = False
            pivoting = False
            publish_motor(client, WF_SPEED, -side * WF_MAX_ANGULAR * 0.5)
            if now - last_status >= STATUS_INTERVAL:
                print(f"  ⚠️  Rear-side contact — driving forward to clear "
                      f"(d_A={'--' if d_a is None else f'{d_a:.2f}m'}, "
                      f"d_B={d_b:.2f}m)")
                last_status = now
            continue

        # Determine distance estimate and heading term.
        #
        # d_C (90°, abeam): sin(90°)=1, so d_C is the exact perpendicular
        # distance.  Use it as d_est whenever available — it's the most
        # accurate reading and the last ray to lose the wall at an outside
        # corner (the wall stays abeam until the robot's centre passes it).
        #
        # d_A (70°) and d_B (110°) are symmetric around 90°, so their
        # difference is zero when parallel → heading_term = d_A - d_B.
        #
        # Ratio check: if d_A and d_B differ by more than RAY_RATIO_THRESHOLD×,
        # one ray has lost the wall; use the closer one for heading.
        RAY_RATIO_THRESHOLD = 2.0

        # Heading term from symmetric 70°/110° pair
        if d_a is not None and d_b is not None:
            ratio = d_b / d_a if d_a > 0 else float('inf')
            if 1.0 / RAY_RATIO_THRESHOLD <= ratio <= RAY_RATIO_THRESHOLD:
                heading_term = d_a - d_b
                mode_ab = f"2ray ratio={ratio:.2f}"
            elif ratio > RAY_RATIO_THRESHOLD:
                heading_term = 0.0   # d_B unreliable
                mode_ab = f"1ray(A) ratio={ratio:.2f}"
            else:
                heading_term = 0.0   # d_A unreliable
                mode_ab = f"1ray(B) ratio={ratio:.2f}"
        elif d_a is not None:
            heading_term = 0.0
            mode_ab = "A-only"
        elif d_b is not None:
            heading_term = 0.0
            mode_ab = "B-only"
        else:
            heading_term = 0.0
            mode_ab = "AB-none"

        # d_est: prefer d_C (exact perpendicular) but ONLY when it's seeing a
        # nearby wall (< LOST_WALL_DIST).  If d_C is far (seeing a distant
        # feature across the room), fall back to the d_A/d_B ratio logic.
        # A 2.77m d_C reading when d_B=0.59m is clearly the wrong wall.
        if d_c is not None and d_c < LOST_WALL_DIST:
            d_est     = d_c                    # sin(90°)=1, exact perpendicular
            wall_lost = d_est > LOST_WALL_DIST # always False here, but for clarity
            single_ray = (d_a is None and d_b is None)
            mode_tag = f"C={d_c:.2f} {mode_ab}"
        elif d_a is not None and d_b is not None:
            ratio = d_b / d_a if d_a > 0 else float('inf')
            if 1.0 / RAY_RATIO_THRESHOLD <= ratio <= RAY_RATIO_THRESHOLD:
                d_est = ((d_a + d_b) / 2.0) * SIN_ANGLE_A
                mode_tag = f"AB {mode_ab}"
            elif ratio > RAY_RATIO_THRESHOLD:
                d_est = d_a * SIN_ANGLE_A
                mode_tag = f"A {mode_ab}"
            else:
                d_est = d_b * SIN_ANGLE_A
                mode_tag = f"B {mode_ab}"
            wall_lost  = d_est > LOST_WALL_DIST
            single_ray = True
        elif d_a is not None:
            d_est      = d_a * SIN_ANGLE_A
            wall_lost  = d_est > LOST_WALL_DIST
            single_ray = True
            mode_tag   = "A-only"
        elif d_b is not None:
            d_est      = d_b * SIN_ANGLE_A
            wall_lost  = d_est > LOST_WALL_DIST
            single_ray = True
            mode_tag   = "B-only"
        else:
            d_est      = None
            wall_lost  = True
            single_ray = False
            mode_tag   = "all-none"

        # ── DIAGNOSTIC: print every scan when d_est is large or wall_lost ──
        if wall_lost or (d_est is not None and d_est > WF_TARGET_DIST * 1.2):
            print(f"  🔍 d_A={d_a if d_a is None else f'{d_a:.2f}'}  "
                  f"d_C={d_c if d_c is None else f'{d_c:.2f}'}  "
                  f"d_B={d_b if d_b is None else f'{d_b:.2f}'}  "
                  f"d_est={d_est if d_est is None else f'{d_est:.2f}'}  "
                  f"mode={mode_tag}  wall_lost={wall_lost}  "
                  f"cooldown_ok={now>=corner_arc_cooldown_until}  "
                  f"arc_ok={corner_arc_until<=now}  "
                  f"recent_wall={now-last_close_wall_t:.1f}s")

        # --- Corner-recovery: reverse then pivot ---
        # When an outside corner end-face is detected (front blocked + wall
        # lost), initiate a forward arc to swing the nose clear.
        # The check at the TOP of the loop handles execution.
        if now < reverse_until:
            continue   # legacy — should not be reached; pivot_lock_until covers it

        # --- Inside corner: front blocked ---
        if front_d is not None and front_d < DANGER_DIST:
            if wall_lost:
                # Outside corner end-face: initiate forward arc to swing clear.
                # The top-of-loop check will execute it uninterrupted.
                pivot_lock_until = now + CORNER_RECOVERY_SEC
                print(f"  🔒 Outside corner end-face "
                      f"(front={front_d:.2f}m, wall lost) — "
                      f"forward arc for {CORNER_RECOVERY_SEC:.1f}s")
            else:
                # True inside corner — pivot in place
                pivoting = True
                sweeping = False
                publish_motor(client, 0.0, -side * WF_MAX_ANGULAR)
                if now - last_status >= STATUS_INTERVAL:
                    print(f"  🔄 Inside corner — pivoting "
                          f"(front={front_d:.2f}m)")
                    last_status = now
            continue

        if pivoting:
            pivoting = False
            prev_dist_error = 0.0

        # --- Outside corner: wall lost ---
        if wall_lost:
            # Outside corner signature: d_A gone (front ray passed corner) but
            # d_B still sees the wall (rear ray hasn't reached the corner yet).
            # Execute a geometric 90° arc with radius = current wall distance.
            # This is mathematically exact: after the arc the robot arrives
            # parallel to and at distance d from the new wall.
            # Prefer d_B for distance since d_A is unreliable at this point.
            if (corner_arc_until <= now and
                    now >= corner_arc_cooldown_until and
                    now - last_close_wall_t < ARC_TRIGGER_WINDOW):
                sweeping = False
                # Prefer d_C (exact perpendicular) for arc radius, but only
                # when it's actually seeing the nearby wall (< WF_TARGET_DIST*1.5).
                # A far d_C reading (e.g. 2.77m) means the 90° ray is hitting
                # a distant feature and should be ignored for radius calculation.
                if d_c is not None and d_c < WF_TARGET_DIST * 1.5 and d_c >= CORNER_ARC_MIN_RADIUS:
                    d_perp = d_c
                else:
                    d_perp_b = d_b * SIN_ANGLE_A if d_b is not None else None
                    if d_perp_b is not None and CORNER_ARC_MIN_RADIUS <= d_perp_b < WF_TARGET_DIST * 1.5:
                        d_perp = d_perp_b
                    else:
                        d_perp = WF_TARGET_DIST
                corner_arc_angular = min(WF_SPEED / d_perp, WF_MAX_ANGULAR)
                arc_duration = (math.pi / 2) / corner_arc_angular
                corner_arc_until = now + arc_duration
                print(f"  🔄 Outside corner: radius={d_perp:.2f}m  "
                      f"ω={corner_arc_angular:.2f}rad/s  "
                      f"duration={arc_duration:.1f}s")
                continue

            # Fallback sweep: both rays lost or arc already attempted —
            # hunt gently for the wall.  But first: if we just finished an
            # arc, drive straight for POST_ARC_STRAIGHT_SEC so the new wall
            # converges naturally without sweep rotation adding overshoot.
            if now < post_arc_straight_until:
                publish_motor(client, WF_SPEED, 0.0)
                if now - last_status >= STATUS_INTERVAL:
                    print(f"  ➡️  Post-arc straight "
                          f"({post_arc_straight_until - now:.1f}s left, "
                          f"d_C={'--' if d_c is None else f'{d_c:.2f}m'})")
                    last_status = now
                continue
            if not sweeping:
                sweeping    = True
                sweep_start = now
                print(f"  ↪️  Wall lost "
                      f"(d_A={'None' if d_a is None else f'{d_a:.2f}m'}, "
                      f"d_B={'None' if d_b is None else f'{d_b:.2f}m'}"
                      f"{'' if d_est is None else f', d_est={d_est:.2f}m'}) "
                      f"— sweeping toward {side_name}")
            if now - sweep_start > SWEEP_TIMEOUT:
                publish_motor(client, WF_SPEED, 0.0)
            else:
                publish_motor(client, WF_SPEED * 0.5,
                              side * WF_MAX_ANGULAR * SWEEP_ANGULAR_RATE)
            continue
        else:
            if sweeping:
                print(f"  ✅ Wall reacquired (d_est={d_est:.2f}m)")
            sweeping = False
            # Track last time we had a confirmed close wall reading
            if d_est is not None and d_est < LOST_WALL_DIST:
                last_close_wall_t = now

        # --- Proactive outside corner detection ---
        # Three paths to detect the corner before wall_lost/sweep:

        # Path 1: d_A large while d_B still close (single-B mode)
        # The ratio check guarantees d_A >> d_B; check d_A is absolutely
        # large (> 2× expected) to confirm the front ray passed the corner.
        _expected_d_a = WF_TARGET_DIST / SIN_ANGLE_A  # ≈ 0.43m at target
        if (single_ray and
                d_a is not None and d_a > _expected_d_a * 2.0 and
                d_b is not None and
                corner_arc_until <= now and
                now >= corner_arc_cooldown_until):
            d_perp_b = d_b * SIN_ANGLE_A
            d_perp   = d_perp_b if d_perp_b >= CORNER_ARC_MIN_RADIUS else WF_TARGET_DIST
            corner_arc_angular = min(WF_SPEED / d_perp, WF_MAX_ANGULAR)
            arc_duration       = (math.pi / 2) / corner_arc_angular
            corner_arc_until   = now + arc_duration
            sweeping = False
            print(f"  🔄 Outside corner (d_A={d_a:.2f}m large, d_B={d_b:.2f}m close): "
                  f"radius={d_perp:.2f}m  ω={corner_arc_angular:.2f}rad/s  "
                  f"duration={arc_duration:.1f}s")
            continue

        # Path 2: d_A just went None while d_C still sees the nearby wall
        if (single_ray and d_a is None and
                d_c is not None and d_c < LOST_WALL_DIST and
                corner_arc_until <= now and
                now >= corner_arc_cooldown_until):
            d_perp_c = d_c   # sin(90°)=1, direct perpendicular distance
            if d_perp_c >= CORNER_ARC_MIN_RADIUS:
                corner_arc_angular = min(WF_SPEED / d_perp_c, WF_MAX_ANGULAR)
                arc_duration = (math.pi / 2) / corner_arc_angular
                corner_arc_until = now + arc_duration
                sweeping = False
                print(f"  🔄 Outside corner (early, d_C={d_c:.2f}m): "
                      f"radius={d_perp_c:.2f}m  "
                      f"ω={corner_arc_angular:.2f}rad/s  "
                      f"duration={arc_duration:.1f}s")
                continue

        # Path 3: d_A None, d_B close (no d_C)
        if (single_ray and d_a is None and d_b is not None and d_c is None and
                corner_arc_until <= now and
                now >= corner_arc_cooldown_until):
            d_perp_b = d_b * SIN_ANGLE_A
            if CORNER_ARC_MIN_RADIUS <= d_perp_b < WF_TARGET_DIST * 1.5:
                d_perp = d_perp_b
            elif d_perp_b >= WF_TARGET_DIST * 1.5:
                d_perp = WF_TARGET_DIST
            else:
                d_perp = None
            if d_perp is not None:
                corner_arc_angular = min(WF_SPEED / d_perp, WF_MAX_ANGULAR)
                arc_duration = (math.pi / 2) / corner_arc_angular
                corner_arc_until = now + arc_duration
                sweeping = False
                print(f"  🔄 Outside corner (early, d_B fallback): "
                      f"radius={d_perp:.2f}m  "
                      f"ω={corner_arc_angular:.2f}rad/s  "
                      f"duration={arc_duration:.1f}s")
                continue

        # --- Normal following: two-ray heading + distance PD ---
        dist_error      = d_est - WF_TARGET_DIST
        dist_d_error    = (dist_error - prev_dist_error) / dt
        prev_dist_error = dist_error

        angular = side * (KP * dist_error + KD * dist_d_error
                          + KH * heading_term)
        angular = max(-WF_MAX_ANGULAR, min(WF_MAX_ANGULAR, angular))

        publish_motor(client, WF_SPEED, angular)

        if now - last_status >= STATUS_INTERVAL:
            ray_info = (f"d_A={d_a:.2f}m d_C={d_c if d_c is None else f'{d_c:.2f}m'} d_B={d_b:.2f}m" if not single_ray
                        else f"d_A={'--' if d_a is None else f'{d_a:.2f}m'} "
                             f"d_C={'--' if d_c is None else f'{d_c:.2f}m'} "
                             f"d_B={'--' if d_b is None else f'{d_b:.2f}m'} "
                             f"[single-ray]")
            print(f"  ↔  d_est={d_est:.2f}m  err={dist_error:+.2f}m  "
                  f"{ray_info}  "
                  f"heading={heading_term:+.2f}  "
                  f"front={'--' if front_d is None else f'{front_d:.2f}m'}  "
                  f"ω={angular:+.2f}")
            last_status = now


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    global WF_SPEED
    parser = argparse.ArgumentParser(
        description="Drive to waypoint then follow wall")
    parser.add_argument("--broker",   default=BROKER)
    parser.add_argument("--waypoints", nargs="+", type=float,
                        metavar="N",
                        default=[-0.90, 2.90],
                        help="One or more waypoints as x1 y1 x2 y2 ... "
                             "(default: -0.90 2.90)")
    parser.add_argument("--side",     choices=["left", "right"],
                        default="left",
                        help="Wall side to follow after waypoint "
                             "(default: left)")
    parser.add_argument("--align-heading", type=float, default=None,
                        metavar="DEG",
                        help="Rotate to this heading (degrees, math convention: "
                             "0=east, 90=north, 180=west) before starting "
                             "wall-following. Use when waypoint delivery leaves "
                             "the robot at the wrong angle for wall acquisition.")
    parser.add_argument("--speed",    type=float, default=WF_SPEED,
                        help=f"Wall-following speed m/s (default: {WF_SPEED})")
    args = parser.parse_args()

    raw = args.waypoints
    if len(raw) % 2 != 0:
        print("❌ --waypoints must be pairs of x y values")
        return
    waypoints = [(raw[i], raw[i+1]) for i in range(0, len(raw), 2)]
    WF_SPEED = args.speed

    # Connect MQTT
    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = _on_connect
    client.on_message = _on_message
    client.connect(args.broker, BROKER_PORT, 60)
    client.loop_start()

    print(f"Connecting to {args.broker}...")
    time.sleep(1.0)

    try:
        # Phase 1 — wait for pose, drive to waypoint
        if not wait_for_pose():
            print("❌ No pose received — is icp_localizer running?")
            return

        print(f"📍 Phase 1 — driving through {len(waypoints)} waypoint(s)")
        for i, (wx, wy) in enumerate(waypoints):
            print(f"   Waypoint {i+1}/{len(waypoints)}: ({wx:.3f}, {wy:.3f})")
            drive_to_waypoint(client, wx, wy)

        # Brief pause at waypoint before handing off
        time.sleep(0.5)

        # Optional heading alignment before wall-following starts
        if args.align_heading is not None:
            rotate_to_heading(client, args.align_heading)
            time.sleep(0.3)

        # Signal obstacle_avoidance to exit before starting wall-following.
        # Both processes publish to MOTOR_CMD; running simultaneously causes
        # the stuttering / spasmodic-lurch problem.
        print("\n📢 Sending shutdown signal to obstacle_avoidance...")
        client.publish(Topics.NAV_SHUTDOWN, "shutdown")
        time.sleep(0.5)   # give obstacle_avoidance time to exit cleanly

        print(f"\n🤝 Handoff → wall-following ({args.side} wall)")

        # Phase 2 — wall following (no keepalive needed — OA has exited)
        follow_walls(client, args.side)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        publish_motor(client, 0.0, 0.0)
        publish_nav(client, 0.0, 0.0)
        time.sleep(0.2)
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
