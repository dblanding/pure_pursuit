#!/usr/bin/env python3
"""
Create map metadata for path planning
Ensure metadata values are same as those used in map creation
"""
import json

metadata = {
    "resolution": 0.05,      # meters per cell
    "width": 300,            # cells
    "height": 300,           # cells
    "origin_x": -3.0,        # meters
    "origin_y": -7.0,        # meters
    "occupied_threshold": 0.65,
    "free_threshold": 0.35,
    "robot_radius": 0.20,    # meters
    "map_files": {
        "occupancy": "map_clean.png",
        "binary": "map_binary.png",
        "planning": "map_planning.png"
    }
}

with open('map_metadata.json', 'w') as f:
    json.dump(metadata, f, indent=2)

print("✅ Created map_metadata.json")
print(json.dumps(metadata, indent=2))
