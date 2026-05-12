#!/usr/bin/env python3
# test_x_direction.py

import json
import time

# Create a path that moves ONLY in +X direction
path_data = {
    "path": [
        [1.0, 2.0],  # Start
        [2.0, 2.0]   # Goal (1 meter forward in X, no change in Y)
    ],
    "num_waypoints": 2,
    "start": [1.0, 2.0],
    "goal": [2.0, 2.0],
    "total_distance": 1.0,
    "timestamp": time.time()
}

with open('test_x_path.json', 'w') as f:
    json.dump(path_data, f, indent=2)

print("✅ Created test_x_path.json")
print("   This path moves 1 meter in +X direction (robot forward)")
print("\nRun: uv run path_follower.py --path-file test_x_path.json")
