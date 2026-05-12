#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import time

client = mqtt.Client()
client.connect("raspibot.local", 1883, 60)
client.loop_start()

print("🧭 Testing robot coordinate system:")
print("=" * 60)

# Test 1: Move "forward" (what the robot thinks is forward)
print("\n1️⃣ Sending: linear=0.3 m/s for 2 seconds")
print("   → Watch which direction the robot moves")
print("   → Is this +X or +Y on your map?")
input("   Press ENTER to start...")

client.publish("robot/motor/cmd", json.dumps({"linear": 0.3, "angular": 0.0}))
time.sleep(2)
client.publish("robot/motor/cmd", json.dumps({"linear": 0.0, "angular": 0.0}))

print("\n   ❓ Did the robot move in +X or +Y direction on the map?")
direction = input("   Type 'X' or 'Y': ").strip().upper()

print(f"\n   📝 Robot's forward = Map's +{direction} direction")

client.loop_stop()
