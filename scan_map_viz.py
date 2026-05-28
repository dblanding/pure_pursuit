# scan_map_viz.py
# Run with robot stationary at home. Shows scan points overlaid on survey map.
import json, math, time
import numpy as np
from PIL import Image, ImageDraw
import paho.mqtt.client as mqtt
import threading
import sys

sys.path.insert(0, '../raspibot/robot')
from topics import Topics

BROKER = "192.168.1.85"
MAP_FILE = "survey_planning_base.png"
RESOLUTION = 0.05
ORIGIN_X, ORIGIN_Y = -3.0, -7.0
WIDTH_PX = HEIGHT_PX = 300

def m_to_px(x, y):
    col = int((x - ORIGIN_X) / RESOLUTION)
    row = HEIGHT_PX - 1 - int((y - ORIGIN_Y) / RESOLUTION)
    return col, row

received = threading.Event()
scan_hold = [None]

def on_msg(client, userdata, msg):
    scan_hold[0] = json.loads(msg.payload.decode())
    received.set()

client = mqtt.Client()
client.username_pw_set("robot", "robot")
client.message_callback_add(Topics.LIDAR_SCAN, on_msg)
client.connect(BROKER, 1883, 60)
client.loop_start()
client.subscribe(Topics.LIDAR_SCAN)
print("Waiting for scan...")
received.wait(timeout=5)
client.loop_stop(); client.disconnect()

scan = scan_hold[0]
if not scan:
    print("No scan received"); exit()

# Convert scan to map frame assuming robot at (0, 0, 0)
img = Image.open(MAP_FILE).convert("RGB")
draw = ImageDraw.Draw(img)

# Mark home position
hx, hy = m_to_px(0, 0)
draw.ellipse([hx-4, hy-4, hx+4, hy+4], fill=(0, 0, 255))

# Plot each scan point
for entry in scan:
    a = entry["a"]   # CCW positive (already negated by scanner.py)
    d = entry["d"]
    if 0.05 < d < 5.0:
        # Current formula
        x = d * math.cos(a)
        y = d * math.sin(a)
        col, row = m_to_px(x, y)
        if 0 <= col < WIDTH_PX and 0 <= row < HEIGHT_PX:
            draw.point((col, row), fill=(255, 0, 0))

img.save("scan_overlay.png")
print("Saved scan_overlay.png — red=scan points, blue=home position")
print("Check if red points align with black map walls")
