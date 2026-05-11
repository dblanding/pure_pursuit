#!/usr/bin/env python3
"""
motor_control_service.py - MQTT subscriber for motor commands
Runs on RASPBERRY PI as a service
Subscribes to: Topics.MOTOR_CMD
"""
import paho.mqtt.client as mqtt
import serial
import json
import time
import sys
#sys.path.insert(0, '../raspibot/robot')  # path to where topics.py is
from topics import Topics

class MotorControl:
    """
    MQTT to Serial bridge for motor control
    """
    
    def __init__(self, serial_port='/dev/serial0', baudrate=115200):
        self.running = False
        self.serial_port = serial_port
        self.baudrate = baudrate
        
        # Serial connection to motor controller
        print(f"🔌 Connecting to serial port {serial_port}...")
        try:
            self.ser = serial.Serial(serial_port, baudrate, timeout=1)
            time.sleep(0.1)
            print(f"✅ Serial connected")
        except Exception as e:
            print(f"❌ Serial connection failed: {e}")
            sys.exit(1)
        
        # MQTT client
        print(f"🔌 Connecting to MQTT broker (localhost)...")
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        try:
            self.client.connect('localhost', 1883, 60)
            print(f"✅ MQTT connected")
        except Exception as e:
            print(f"❌ MQTT connection failed: {e}")
            sys.exit(1)
        
        print(f"\n{'='*60}")
        print(f"🤖 Motor Control Service Ready")
        print(f"{'='*60}")
        print(f"Subscribed to:")
        print(f"  - {Topipcs.MOTOR_CMD} (velocity commands)")
        #print(f"  - robot/motor/service/control (start/stop)")
        #print(f"\nService state: STOPPED (waiting for START command)")
        print(f"{'='*60}\n")
        
        # Start MQTT loop
        self.client.loop_forever()
    
    def on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
        if rc == 0:
            # Subscribe to motor commands and service control
            client.subscribe("robot/motor/cmd")
            client.subscribe("robot/motor/service/control")
        else:
            print(f"❌ MQTT connection failed with code {rc}")
            sys.exit(1)
    
    def on_message(self, client, userdata, msg):
        """MQTT message callback"""
        try:
            # Parse JSON payload
            data = json.loads(msg.payload.decode())
            
            # Handle service control messages
            if msg.topic == "robot/motor/service/control":
                action = data.get('action', '')
                
                if action == 'start':
                    self.running = True
                    # Set motor controller to AUTO mode
                    self.ser.write(b"M,AUTO\n")
                    time.sleep(0.05)
                    print(f"✅ Service STARTED (AUTO mode)")
                    
                elif action == 'stop':
                    self.running = False
                    # Send emergency stop
                    self.ser.write(b"S\n")
                    time.sleep(0.05)
                    print(f"🛑 Service STOPPED (motors stopped)")
                
                else:
                    print(f"⚠️  Unknown action: {action}")
            
            # Handle motor command messages
            elif msg.topic == "robot/motor/cmd":
                # Only process commands if service is running
                if not self.running:
                    return
                
                # Extract velocity commands
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
                
                # Format serial command: V,linear,angular
                cmd = f"V,{linear:.3f},{angular:.3f}\n"
                
                # Send to motor controller
                self.ser.write(cmd.encode())
                
                # Optional: print received commands (comment out for less verbose output)
                # print(f"📤 v={linear:6.3f} m/s, ω={angular:6.3f} rad/s")
        
        except Exception as e:
            print(f"⚠️  Error processing message: {e}")
            print(f"    Topic: {msg.topic}")
            print(f"    Payload: {msg.payload.decode()}")


if __name__ == '__main__':
    try:
        service = MotorControlService(
            serial_port='/dev/serial0',
            baudrate=115200
        )
    except KeyboardInterrupt:
        print(f"\n👋 Service stopped by user")
        sys.exit(0)
