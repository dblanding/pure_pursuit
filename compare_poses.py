#!/usr/bin/env python3
# compare_poses.py
"""
Subscribes to both odometry and localization pose topics and logs them
to a timestamped CSV file for analysis in a spreadsheet.

Also prints a compact summary line to the terminal so you can see
what's happening in real time without flooding the screen.

Usage:
    python compare_poses.py
    python compare_poses.py --broker 192.168.1.85

CSV columns:
    elapsed_s   — seconds since script started
    odom_x, odom_y, odom_h_deg
    map_x,  map_y,  map_h_deg
    delta_x, delta_y, delta_h_deg   — map minus odom (the ICP correction)

Terminal output (one line, updated every PRINT_INTERVAL_SEC seconds):
    t=12.3s  ODOM x=+0.1234 y=-0.0567 h=+2.31°  |  MAP x=+0.1012 y=-0.0489 h=+2.18°  |  Δ dx=-0.022 dy=+0.008 dh=-0.13°
"""

import sys
import csv
import json
import math
import time
import argparse
import threading
import paho.mqtt.client as mqtt
from datetime import datetime

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_BROKER    = "raspibot.local"
BROKER_PORT       = 1883
MQTT_USERNAME     = "robot"
MQTT_PASSWORD     = "robot"
PRINT_INTERVAL_SEC = 1.0    # how often to print a summary line to terminal

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
latest    = {"odom": None, "map": None}
lock      = threading.Lock()
t_start   = time.monotonic()
last_print = [0.0]

CSV_FILE  = f"log_files/pose_compare_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
CSV_COLS  = ["elapsed_s",
             "odom_x", "odom_y", "odom_h_deg",
             "map_x",  "map_y",  "map_h_deg",
             "delta_x", "delta_y", "delta_h_deg"]

log_fh  = open(CSV_FILE, "w", newline="")
log_csv = csv.writer(log_fh)
log_csv.writerow(CSV_COLS)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def wrap_deg(d):
    """Wrap angle to [-180, 180]."""
    return (d + 180) % 360 - 180


def log_and_print():
    """Write current state to CSV and maybe print a terminal summary."""
    with lock:
        odom = latest["odom"]
        mapf = latest["map"]

    if odom is None or mapf is None:
        return

    elapsed = round(time.monotonic() - t_start, 3)
    oh      = math.degrees(odom["h"])
    mh      = math.degrees(mapf["h"])
    dx      = mapf["x"] - odom["x"]
    dy      = mapf["y"] - odom["y"]
    dh      = wrap_deg(mh - oh)

    # Write to CSV every message
    log_csv.writerow([
        elapsed,
        round(odom["x"], 6), round(odom["y"], 6), round(oh, 4),
        round(mapf["x"], 6), round(mapf["y"], 6), round(mh, 4),
        round(dx, 6),        round(dy, 6),         round(dh, 4),
    ])
    log_fh.flush()

    # Print summary at reduced rate so terminal stays readable
    now = time.monotonic()
    if now - last_print[0] >= PRINT_INTERVAL_SEC:
        last_print[0] = now
        print(
            f"t={elapsed:6.1f}s  "
            f"ODOM x={odom['x']:+.4f} y={odom['y']:+.4f} h={oh:+.2f}°  |  "
            f"MAP  x={mapf['x']:+.4f} y={mapf['y']:+.4f} h={mh:+.2f}°  |  "
            f"Δ dx={dx:+.4f} dy={dy:+.4f} dh={dh:+.3f}°"
        )


# ---------------------------------------------------------------------------
# MQTT callbacks
# ---------------------------------------------------------------------------

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(Topics.ODOM_POSE)
        client.subscribe(Topics.POSE)
        print(f"Connected. Logging to {CSV_FILE}")
        print(f"Monitoring:  {Topics.ODOM_POSE}  |  {Topics.POSE}")
        print()
    else:
        print(f"Connection failed, rc={rc}")


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        pose = {
            "x": float(data["x"]),
            "y": float(data["y"]),
            "h": float(data["h"]),
        }
        with lock:
            if msg.topic == Topics.ODOM_POSE:
                latest["odom"] = pose
            elif msg.topic == Topics.POSE:
                latest["map"] = pose
        log_and_print()
    except Exception as e:
        print(f"Parse error on {msg.topic}: {e}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Side-by-side pose logger")
    parser.add_argument("--broker", default=DEFAULT_BROKER,
                        help=f"MQTT broker address (default: {DEFAULT_BROKER})")
    args = parser.parse_args()

    client = mqtt.Client()
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to {args.broker}:{BROKER_PORT}...")
    client.connect(args.broker, BROKER_PORT, 60)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print(f"\nStopped. CSV saved to {CSV_FILE}")
    finally:
        log_fh.close()
        client.disconnect()
