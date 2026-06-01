#!/usr/bin/env python3
# test_x_direction.py

import json
import time

# Create a path that moves ONLY in +X direction
path_data = {
    "path": [
        [0.0, 0.0],  # Start
        [2.5, 0.0]   # Goal (2.5 meters forward in X, no change in Y)
    ],
    "num_waypoints": 2,
    "start": [0.0, 0.0],
    "goal": [2.5, 0.0],
    "total_distance": 2.5,
    "timestamp": time.time()
}

with open('test_x_path.json', 'w') as f:
    json.dump(path_data, f, indent=2)

print("✅ Created test_x_path.json")
print("   This path moves 1 meter in +X direction (robot forward)")
print("\nRun: uv run path_follower.py --path-file test_x_path.json")
