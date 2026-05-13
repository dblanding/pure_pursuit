"""
Localization - Transforms odometry to map frame

Phase 1: Simple transform based on initial pose
Future: Will integrate particle filter for drift correction
"""

import json
import time
import math
from threading import Lock
import paho.mqtt.client as mqtt
from map_utils import get_home_position
from mqtt_utils import connect_mqtt_client
import sys

sys.path.insert(0, '../raspibot/robot')  # path to where topics.py is
from topics import Topics

class Localization:
    """Transforms odometry (odom frame) to pose (map frame)"""
    
    def __init__(self):
        # Transform: map = odom + offset
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_theta = 0.0
        self.offset_lock = Lock()
        
        # Current odometry (for calculating offset)
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_theta = 0.0
        self.current_xr = 0.0  # velocity rates from IMU
        self.current_yr = 0.0
        self.current_hr = 0.0
        self.odom_lock = Lock()
        
        self.initialized = False
        
        # MQTT setup
        self.client = mqtt.Client()
        broker = connect_mqtt_client(
            self.client,
            on_connect=self.on_connect,
            on_message=self.on_message
        )
        
        self.client.loop_start()
        
        print("🗺️  Localization started")
        print(f"   Connected to MQTT broker: {broker}")
        print("   Waiting for initial pose...")
        print(f"   Send to topic: {Topics.INITIAL_POSE}")
    
    def on_connect(self, client, userdata, flags, rc):
        """Subscribe to topics on connection"""
        if rc == 0:
            client.subscribe(Topics.ODOM_POSE)
            client.subscribe(Topics.INITIAL_POSE)
            print("✅ Connected to MQTT broker")
        else:
            print(f"❌ Connection failed with code {rc}")
    
    def on_message(self, client, userdata, msg):
        """Handle incoming messages"""
        try:
            data = json.loads(msg.payload)
            
            if msg.topic == Topics.ODOM_POSE:
                self.handle_odometry(data)
            
            elif msg.topic == Topics.INITIAL_POSE:
                self.handle_initial_pose(data)
        
        except json.JSONDecodeError as e:
            print(f"⚠️  Invalid JSON on {msg.topic}: {e}")
        except Exception as e:
            print(f"⚠️  Error processing message: {e}")
    
    def handle_odometry(self, odom_data):
        """Process raw odometry and publish transformed pose"""
        # Update current odometry
        with self.odom_lock:
            self.current_odom_x = odom_data['x']
            self.current_odom_y = odom_data['y']
            self.current_odom_theta = odom_data['h']
            # Store velocity rates (for future particle filter use)
            self.current_xr = odom_data.get('xr', 0.0)
            self.current_yr = odom_data.get('yr', 0.0)
            self.current_hr = odom_data.get('hr', 0.0)
        
        # Only publish if initialized
        if not self.initialized:
            return
        
        # Transform to map frame
        with self.offset_lock:
            map_x = self.current_odom_x + self.offset_x
            map_y = self.current_odom_y + self.offset_y
            map_theta = self.current_odom_theta + self.offset_theta
        
        # Normalize theta to [-pi, pi]
        map_theta = math.atan2(math.sin(map_theta), math.cos(map_theta))
        
        # Publish localized pose (include velocities)
        pose = {
            'x': map_x,
            'y': map_y,
            'h': map_theta,
            'xr': self.current_xr,
            'yr': self.current_yr,
            'hr': self.current_hr
        }
        self.client.publish(Topics.POSE, json.dumps(pose))
    
    def handle_initial_pose(self, pose_data):
        """Set initial pose in map frame"""
        map_x = pose_data['x']
        map_y = pose_data['y']
        map_theta = pose_data.get('h', pose_data.get('theta', 0.0))  # Accept both 'h' and 'theta'
        
        # Get current odometry
        with self.odom_lock:
            odom_x = self.current_odom_x
            odom_y = self.current_odom_y
            odom_theta = self.current_odom_theta
        
        # Calculate offset: map = odom + offset
        # Therefore: offset = map - odom
        with self.offset_lock:
            self.offset_x = map_x - odom_x
            self.offset_y = map_y - odom_y
            self.offset_theta = map_theta - odom_theta
        
        self.initialized = True
        
        print(f"\n✅ Initial pose set:")
        print(f"   Map frame: ({map_x:.3f}, {map_y:.3f}, {math.degrees(map_theta):.1f}°)")
        print(f"   Odom frame: ({odom_x:.3f}, {odom_y:.3f}, {math.degrees(odom_theta):.1f}°)")
        print(f"   Offset: ({self.offset_x:.3f}, {self.offset_y:.3f}, {math.degrees(self.offset_theta):.1f}°)")
        print(f"   Publishing to: {Topics.POSE}\n")
    
    def run(self):
        """Main loop"""
        try:
            while True:
                time.sleep(1)
                # Future: particle filter update will go here
        
        except KeyboardInterrupt:
            print("\n🛑 Localization stopped")
            self.client.loop_stop()
            self.client.disconnect()

if __name__ == "__main__":
    try:
        loc = Localization()
        loc.run()
    except Exception as e:
        print(f"❌ Localization failed: {e}")
