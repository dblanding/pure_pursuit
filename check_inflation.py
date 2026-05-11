#!/usr/bin/env python3
"""
Check obstacle inflation
"""
import cv2
import numpy as np

# Load maps
binary = cv2.imread('map_binary.png', cv2.IMREAD_GRAYSCALE)
planning = cv2.imread('map_planning.png', cv2.IMREAD_GRAYSCALE)

print(f"Binary map - Free cells: {np.sum(binary > 127)}")
print(f"Planning map - Free cells: {np.sum(planning > 127)}")
print(f"Inflation removed: {np.sum(binary > 127) - np.sum(planning > 127)} cells")

# Show side-by-side
comparison = np.hstack([binary, planning])
cv2.imwrite('inflation_check.png', comparison)
print("✅ Saved inflation_check.png")
