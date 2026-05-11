#!/usr/bin/env python3
"""
High-level navigation controller for home-base mode
"""
import numpy as np
import time
from path_planner import PathPlanner
from path_follower import PathFollower


class RobotNavigator:
    """
    Manages robot navigation missions with automatic return to home
    """
    
    def __init__(self, map_file, robot_interface):
        """
        Args:
            map_file: Path to map YAML file
            robot_interface: Object that provides get_pose() and set_velocity()
        """
        # Home pose (where robot is placed at startup)
        self.home_pose = (0.0, 0.0, 0.0)  # x, y, theta
        self.current_pose = self.home_pose
        
        # Initialize planner and follower
        self.planner = PathPlanner(map_file)
        self.follower = PathFollower(
            max_linear_vel=0.2,
            max_angular_vel=0.8,
            lookahead_distance=0.4,
            goal_tolerance=0.15,
            update_rate=10
        )
        
        # Robot hardware interface
        self.robot = robot_interface
        
        print(f"🏠 Robot initialized at home: {self.home_pose}")
    
    def go_to_goal_and_return(self, goal_x, goal_y):
        """
        Drive to goal, then return to home
        
        Args:
            goal_x, goal_y: Goal position in meters
        
        Returns:
            True if mission successful, False otherwise
        """
        print(f"\n{'='*50}")
        print(f"🚀 MISSION START: Go to ({goal_x:.2f}, {goal_y:.2f})")
        print(f"{'='*50}")
        
        # Phase 1: Drive to goal
        print(f"\n📍 Phase 1: Driving to goal...")
        success = self._drive_to_goal(goal_x, goal_y)
        
        if not success:
            print(f"❌ Failed to reach goal!")
            self._emergency_stop()
            return False
        
        print(f"✅ Reached goal!")
        time.sleep(1.0)  # Pause at goal
        
        # Phase 2: Return to home
        print(f"\n📍 Phase 2: Returning to home...")
        success = self._return_to_home()
        
        if not success:
            print(f"❌ Failed to return home!")
            self._emergency_stop()
            return False
        
        print(f"✅ Returned to home!")
        
        # Reset pose to home (correct any odometry drift)
        self.current_pose = self.home_pose
        
        print(f"\n{'='*50}")
        print(f"🎉 MISSION COMPLETE")
        print(f"{'='*50}\n")
        
        return True
    
    def _drive_to_goal(self, goal_x, goal_y):
        """
        Plan and execute path to goal
        
        Returns:
            True if successful
        """
        # Get current pose from robot
        self.current_pose = self.robot.get_pose()
        robot_x, robot_y, robot_theta = self.current_pose
        
        # Plan path
        print(f"🗺️  Planning path from ({robot_x:.2f}, {robot_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})...")
        path = self.planner.plan(robot_x, robot_y, goal_x, goal_y)
        
        if path is None:
            print(f"❌ No path found!")
            return False
        
        # Follow path
        self.follower.set_path(path)
        return self._execute_path(max_time=60.0)
    
    def _return_to_home(self):
        """
        Plan and execute path back to home
        
        Returns:
            True if successful
        """
        # Get current pose
        self.current_pose = self.robot.get_pose()
        robot_x, robot_y, robot_theta = self.current_pose
        
        home_x, home_y, _ = self.home_pose
        
        # Plan return path
        print(f"🗺️  Planning return path to home ({home_x:.2f}, {home_y:.2f})...")
        path = self.planner.plan(robot_x, robot_y, home_x, home_y)
        
        if path is None:
            print(f"❌ No return path found!")
            return False
        
        # Follow path home
        self.follower.set_path(path)
        return self._execute_path(max_time=60.0)
    
    def _execute_path(self, max_time=60.0):
        """
        Execute current path using path follower
        
        Args:
            max_time: Maximum time to execute (seconds)
        
        Returns:
            True if goal reached, False if timeout or error
        """
        start_time = time.time()
        
        try:
            while True:
                # Check timeout
                if time.time() - start_time > max_time:
                    print(f"⏱️  Timeout after {max_time}s")
                    self._emergency_stop()
                    return False
                
                # Get current robot pose
                robot_x, robot_y, robot_theta = self.robot.get_pose()
                self.current_pose = (robot_x, robot_y, robot_theta)
                
                # Get velocity command
                cmd = self.follower.get_velocity_command(robot_x, robot_y, robot_theta)
                
                if cmd is None:
                    # No path set
                    self._emergency_stop()
                    return False
                
                linear_vel, angular_vel = cmd
                
                # Check if goal reached
                if linear_vel == 0.0 and angular_vel == 0.0:
                    self.robot.set_velocity(0.0, 0.0)
                    return True
                
                # Send velocity command to robot
                self.robot.set_velocity(linear_vel, angular_vel)
                
                # Sleep to maintain update rate
                time.sleep(self.follower.dt)
        
        except KeyboardInterrupt:
            print(f"\n⚠️  Interrupted by user!")
            self._emergency_stop()
            return False
    
    def _emergency_stop(self):
        """Stop robot immediately"""
        print(f"🛑 EMERGENCY STOP")
        self.robot.set_velocity(0.0, 0.0)
        self.follower.reset()
    
    def set_home(self, x, y, theta):
        """
        Update home position
        
        Args:
            x, y: Home position (m)
            theta: Home orientation (radians)
        """
        self.home_pose = (x, y, theta)
        self.current_pose = self.home_pose
        print(f"🏠 Home updated to: ({x:.2f}, {y:.2f}, {np.degrees(theta):.1f}°)")


if __name__ == '__main__':
    print("This module requires a robot interface.")
    print("See robot_interface.py for implementation.")
