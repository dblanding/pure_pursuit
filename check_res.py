import cv2
original = cv2.imread('map.png')
cleaned = cv2.imread('map_clean.png')
print(f"Original: {original.shape if original is not None else 'NOT FOUND'}")
print(f"Cleaned: {cleaned.shape if cleaned is not None else 'NOT FOUND'}")
