#!/usr/bin/env python3
"""
Convert cleaned map to binary format for path planning
WITH PROPER OBSTACLE INFLATION
"""
import numpy as np
import cv2

def create_planning_map(input_file='map_clean.png', 
                       robot_radius_meters=0.20,
                       safety_margin_meters=0.10,
                       resolution=0.05):
    """
    Create binary planning map with inflated obstacles
    """
    
    # Load cleaned map
    img = cv2.imread(input_file, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"❌ Could not load {input_file}")
        return
    
    print(f"📊 Loaded map: {img.shape}")
    
    # Create binary map - BE CONSERVATIVE
    # Treat gray (unknown) as obstacles too!
    binary = np.zeros_like(img)
    binary[img > 200] = 255  # Only VERY white pixels are free
    
    print(f"   Initial free space: {np.sum(binary == 255)} cells ({np.sum(binary == 255)/binary.size*100:.1f}%)")
    
    # Calculate total inflation (robot + safety margin)
    total_inflation = robot_radius_meters + safety_margin_meters
    inflation_cells = int(np.ceil(total_inflation / resolution))
    kernel_size = inflation_cells * 2 + 1
    
    print(f"\n🤖 Inflation parameters:")
    print(f"   Robot radius: {robot_radius_meters}m")
    print(f"   Safety margin: {safety_margin_meters}m")
    print(f"   Total inflation: {total_inflation}m = {inflation_cells} cells")
    print(f"   Kernel size: {kernel_size}x{kernel_size}")
    
    # Create circular kernel (better than square)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    
    # Inflate obstacles (erode free space)
    inflated = cv2.erode(binary, kernel, iterations=1)
    
    # Additional pass for small obstacles (chair legs)
    # Use morphological closing to fill small gaps
    small_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    inflated = cv2.morphologyEx(inflated, cv2.MORPH_CLOSE, small_kernel, iterations=2)
    
    # Calculate statistics
    total_cells = binary.size
    free_before = np.sum(binary == 255)
    free_after = np.sum(inflated == 255)
    
    print(f"\n📈 Results:")
    print(f"   Free space before: {free_before} cells ({free_before/total_cells*100:.1f}%)")
    print(f"   Free space after:  {free_after} cells ({free_after/total_cells*100:.1f}%)")
    print(f"   Removed by inflation: {free_before-free_after} cells ({(free_before-free_after)/total_cells*100:.1f}%)")
    
    # Save versions
    cv2.imwrite('map_binary.png', binary)
    cv2.imwrite('map_planning.png', inflated)
    
    # Create visualization showing inflation
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    
    # Show inflated area in red
    inflation_mask = (binary == 255) & (inflated == 0)
    vis[inflation_mask] = [0, 0, 255]  # Red = inflated obstacles
    
    # Show safe free space in green
    vis[inflated == 255] = [0, 255, 0]  # Green = safe to navigate
    
    cv2.imwrite('map_inflation_viz.png', vis)
    
    print(f"\n✅ Saved:")
    print(f"   map_binary.png - Binary map (no inflation)")
    print(f"   map_planning.png - Planning map (INFLATED)")
    print(f"   map_inflation_viz.png - Visualization (red=inflation, green=safe)")
    
    return inflated

if __name__ == '__main__':
    create_planning_map()
