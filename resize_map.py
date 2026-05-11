#!/usr/bin/env python3
"""
Resize map_clean.png back to original 400x400
"""
import cv2

# Load oversized cleaned map
cleaned = cv2.imread('map_clean.png', cv2.IMREAD_GRAYSCALE)
print(f"❌ Current size: {cleaned.shape} (TOO BIG!)")

# Resize to original 400x400
# Use INTER_NEAREST to keep sharp edges (no blurring)
original_size = (400, 400)  # (width, height)
resized = cv2.resize(cleaned, original_size, interpolation=cv2.INTER_NEAREST)

print(f"✅ Resized to: {resized.shape}")

# Save
cv2.imwrite('map_clean.png', resized)
print("✅ Overwrote map_clean.png with correct size")

# Also create a backup
cv2.imwrite('map_clean_400x400.png', resized)
print("✅ Saved backup to map_clean_400x400.png")
