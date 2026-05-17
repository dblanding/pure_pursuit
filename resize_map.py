#!/usr/bin/env python3
"""
Resize map_clean.png back to ORIGINAL_SIZE
"""
import cv2

# New correct size
WIDTH = 300
HEIGHT = 300
ORIGINAL_SIZE = f"{WIDTH}x{HEIGHT}"

# Load oversized cleaned map
cleaned = cv2.imread('map_clean.png', cv2.IMREAD_GRAYSCALE)
print(f"❌ Current size: {cleaned.shape} (TOO BIG!)")

# Resize to ORIGINAL_SIZE
# Use INTER_NEAREST to keep sharp edges (no blurring)
original_size = (WIDTH, HEIGHT)
resized = cv2.resize(cleaned, original_size, interpolation=cv2.INTER_NEAREST)

print(f"✅ Resized to: {resized.shape}")

# Save
cv2.imwrite('map_clean.png', resized)
print("✅ Overwrote map_clean.png with correct size")

# Also create a backup
cv2.imwrite(f'map_clean_{ORIGINAL_SIZE}.png', resized)
print(f"✅ Saved backup to map_clean_{ORIGINAL_SIZE}.png")
