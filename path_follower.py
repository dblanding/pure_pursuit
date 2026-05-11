#!/usr/bin/env python3
"""
path_follower.py - Follows a planned path using pure pursuit controller
Runs on LAPTOP, publishes motor commands via MQTT
"""
import signal
import paho.mqtt.client as mqtt
import numpy as np
import json
import time
from threading import Lock
import sys
sys.path.insert(0, '../raspibot/robot')  # path to where topics.py is
from topics import Topics

# Global robot reference for signal handler
robot_instance = None

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n🛑 Ctrl+C detected - stopping robot...")
    if robot_instance:
        robot_instance.stop()
        time.sleep(0.2)
    print("✅ Robot stopped")
    sys.exit(0)

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)

class RobotInterface:
    """Simple MQTT interface for odometry and motor commands"""
    
    def __init__(self, mqtt_broker='raspibot.local', mqtt_port=1883):
        # Pose from odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_lock = Lock()
        self.pose_updated = False
        
        # MQTT client
        print(f"🔌 Connecting to {mqtt_broker}:{mqtt_port}...")
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        
        self.client.connect(mqtt_broker, mqtt_port, 60)
        self.client.loop_start()
        print(f"✅ MQTT Connected")
        
        # Wait for odometry
        print(f"⏳ Waiting for odometry...")
        start = time.time()
        while not self.pose_updated and (time.time() - start) < 5.0:
            time.sleep(0.1)
        
        if self.pose_updated:
            print(f"✅ Odometry OK: ({self.x:.3f}, {self.y:.3f}, {np.degrees(self.theta):.1f}°)")
        else:
            print(f"⚠️  No odometry received")
    
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(Topics.ODOM_POSE)
    
    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode())
            with self.pose_lock:
                self.x = float(data['x'])
                self.y = float(data['y'])
                self.theta = float(data['h'])
                self.pose_updated = True
        except:
            pass
    
    def get_pose(self):
        """Get current robot pose (x, y, theta)"""
        with self.pose_lock:
            return (self.x, self.y, self.theta)
    
    def set_velocity(self, linear, angular):
        """Publish velocity command to robot"""
        cmd = {
            "linear": round(linear, 4),
            "angular": round(angular, 4),
            "timestamp": time.time()
        }
        self.client.publish(Topics.MOTOR_CMD, json.dumps(cmd))
    
    def stop(self):
        """Stop the robot"""
        self.set_velocity(0.0, 0.0)
    
    def close(self):
        """Clean shutdown"""
        self.stop()
        time.sleep(0.2)
        self.client.loop_stop()
        self.client.disconnect()


class PurePursuitController:
    """Pure pursuit path following controller"""
    
    def __init__(self, lookahead_distance=0.3, max_linear_vel=0.3, max_angular_vel=1.0):
        """
        Args:
            lookahead_distance: Distance ahead to look for target point (meters)
            max_linear_vel: Maximum linear velocity (m/s)
            max_angular_vel: Maximum angular velocity (rad/s)
        """
        self.lookahead = lookahead_distance
        self.max_linear = max_linear_vel
        self.max_angular = max_angular_vel
    
    def find_lookahead_point(self, path, robot_pos):
        """
        Find the lookahead point on the path
        
        Args:
            path: List of (x, y) waypoints
            robot_pos: Current robot position (x, y)
            
        Returns:
            (x, y) of lookahead point, or None if path complete
        """
        rx, ry = robot_pos
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (px, py) in enumerate(path):
            dist = np.hypot(px - rx, py - ry)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Search forward from closest point for lookahead point
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            dist = np.hypot(px - rx, py - ry)
            
            if dist >= self.lookahead:
                return (px, py)
        
        # If no point found, return last point
        if len(path) > 0:
            return path[-1]
        
        return None
    
    def compute_velocity(self, robot_pose, target_point):
        """
        Compute velocity commands to reach target point
        
        Args:
            robot_pose: (x, y, theta) current pose
            target_point: (x, y) target point
            
        Returns:
            (linear_vel, angular_vel) tuple
        """
        rx, ry, rtheta = robot_pose
        tx, ty = target_point
        
        # Vector to target
        dx = tx - rx
        dy = ty - ry
        distance = np.hypot(dx, dy)
        
        # Angle to target
        target_angle = np.arctan2(dy, dx)
        
        # Angle error (wrapped to [-pi, pi])
        angle_error = target_angle - rtheta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        # Compute velocities
        # Linear velocity: slow down as we approach target
        linear_vel = self.max_linear * min(1.0, distance / self.lookahead)
        
        # Reduce linear velocity if we need to turn sharply
        linear_vel *= (1.0 - abs(angle_error) / np.pi)
        
        # Angular velocity: proportional to angle error
        k_angular = 2.0  # Gain for angular control
        angular_vel = k_angular * angle_error
        
        # Clamp velocities
        linear_vel = np.clip(linear_vel, 0.0, self.max_linear)
        angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)
        
        return (linear_vel, angular_vel)


def follow_path(robot, path, controller, goal_tolerance=0.1, rate=10):
    """
    Follow a planned path using pure pursuit
    
    Args:
        robot: RobotInterface instance
        path: List of (x, y) waypoints from path planner
        controller: PurePursuitController instance
        goal_tolerance: Distance to goal to consider complete (meters)
        rate: Control loop rate (Hz)
    
    Returns:
        True if goal reached, False if interrupted
    """
    if len(path) == 0:
        print("❌ Empty path!")
        return False
    
    goal_x, goal_y = path[-1]
    print(f"🎯 Following path to goal: ({goal_x:.2f}, {goal_y:.2f})")
    print(f"   Path has {len(path)} waypoints")
    print(f"   Lookahead: {controller.lookahead:.2f}m")
    print(f"   Max speeds: {controller.max_linear:.2f} m/s, {controller.max_angular:.2f} rad/s")
    print(f"   Goal tolerance: {goal_tolerance:.2f}m")
    print(f"\nPress Ctrl+C to stop\n")
    
    dt = 1.0 / rate
    
    try:
        while True:
            # Get current pose
            x, y, theta = robot.get_pose()
            
            # Check if goal reached
            dist_to_goal = np.hypot(goal_x - x, goal_y - y)
            
            if dist_to_goal < goal_tolerance:
                print(f"\n✅ Goal reached! Final position: ({x:.3f}, {y:.3f})")
                robot.stop()
                return True
            
            # Find lookahead point
            target = controller.find_lookahead_point(path, (x, y))
            
            if target is None:
                print(f"\n⚠️  No lookahead point found")
                robot.stop()
                return False
            
            # Compute velocities
            linear, angular = controller.compute_velocity((x, y, theta), target)
            
            # Send command
            robot.set_velocity(linear, angular)
            
            # Status update
            print(f"Pos: ({x:6.3f}, {y:6.3f}, {np.degrees(theta):6.1f}°) | "
                  f"Target: ({target[0]:6.3f}, {target[1]:6.3f}) | "
                  f"Goal dist: {dist_to_goal:5.3f}m | "
                  f"Cmd: v={linear:5.2f} ω={angular:5.2f}", end='\r')
            
            time.sleep(dt)
    
    except KeyboardInterrupt:
        print(f"\n\n⚠️  Interrupted by user")
        robot.stop()
        return False


if __name__ == '__main__':
    # Example usage
    print("=" * 60)
    print("PATH FOLLOWER - Pure Pursuit Controller")
    print("=" * 60)
    
    # Connect to robot
    robot = RobotInterface(mqtt_broker='raspibot.local')
    
    # Create controller
    controller = PurePursuitController(
        lookahead_distance=0.3,   # reduce from 30cm lookahead
        max_linear_vel=0.3,       # 30 cm/s max speed
        max_angular_vel=1.0       # 1 rad/s max turn rate
    )
    
    # Example path (replace with your planned path)
    # This is a simple square path for testing
    path = [
        (1.0, 0.0),
        (1.5, 0.5),
        (2.0, 1.0),
        #(0.0, 1.0),
        #(0.0, 0.0)
    ]
    
    print("\n⚠️  EXAMPLE PATH LOADED (square)")
    print("Replace this with your actual planned path!")
    
    response = input("\nFollow path? (y/n): ")
    
    if response.lower() == 'y':
        success = follow_path(
            robot=robot,
            path=path,
            controller=controller,
            goal_tolerance=0.1,  # 10cm tolerance
            rate=10              # 10 Hz control loop
        )
        
        if success:
            print("\n🎉 Path following complete!")
        else:
            print("\n❌ Path following failed")
    
    robot.close()
