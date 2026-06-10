#!/usr/bin/env python3
# build_map_helpers.py
"""
Useful helper functions while building a survey map
"""

def dfs(xc, yc, size=2):
    """
    Generate values to "draw_filled_square" using "draw_filled_rect" command.
    args:
        xc: x coord of center in meters
        yc: y coord of center in meters
        size: size of side in pixels (1 pixel = 0.05m)
    return:
        command ready to paste into  build_map.py
    """
    MAP_RES = 0.05  # meters/pixel
    w = MAP_RES * size / 4
    return f"draw_filled_rect(draw, {xc-w:.3f}, {yc-w:.3f},  {xc+w:.3f}, {yc+w:.3f})"

if __name__== "__main__":
    
    print(dfs(1, 1))
