# build_map.py
"""
Surveyed map builder.

Define your walls, doorways, and obstacles as simple geometry below,
then run this script to produce PNG map files that match your existing
map_metadata.json exactly — same resolution, same origin, same size.

Output files (written to the same directory as this script):
    survey_map.png          — the occupancy map  (black walls, white free space)
    survey_map_overlay.png  — survey map blended over your SLAM map so you can
                              see where they agree and where they differ

Coordinate system
-----------------
All coordinates are in METRES in the robot's map frame:
    +x = east (forward from home position — robot faces east at startup)
    +y = north (leftward from home position)
    Origin (0, 0) = robot home position

Wall thickness
--------------
Real walls are ~100–120mm thick. At 5cm/pixel that's 2 pixels.
Interior walls, door frames, and furniture edges can be 1 pixel (5cm).
WALL_THICKNESS_M sets the default; individual items can override it.

How to measure
--------------
Measure from the INSIDE face of each wall (the face that faces the room).
That's what your lidar sees, so that's what should match the scan.
Use a laser tape for best results; a good steel tape works fine too.
"""

import json
import math
import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------
# Map parameters — must match map_metadata.json exactly
# ---------------------------------------------------------------------------
RESOLUTION   = 0.05     # metres per pixel
WIDTH_PX     = 300      # image width  in pixels
HEIGHT_PX    = 300      # image height in pixels
ORIGIN_X     = -3.0     # map-frame x of image left edge
ORIGIN_Y     = -7.0     # map-frame y of image bottom edge

# ---------------------------------------------------------------------------
# Drawing defaults
# ---------------------------------------------------------------------------
WALL_THICKNESS_M  = 0.10    # metres — drawn symmetrically about the line
DOOR_WIDTH_M      = 0.90    # informational only; doors are gaps (not drawn)

# Pixel colours
COL_FREE     = 255   # white  — free space
COL_OCCUPIED = 0     # black  — wall / obstacle
COL_UNKNOWN  = 128   # grey   — unknown (not used in survey map)

# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def m_to_px(x_m, y_m):
    """Convert map-frame (x, y) metres to pixel (col, row).
    Pillow's origin is top-left; map origin is bottom-left, so row is flipped.
    """
    col = int((x_m - ORIGIN_X) / RESOLUTION)
    row = HEIGHT_PX - 1 - int((y_m - ORIGIN_Y) / RESOLUTION)
    return col, row


def thickness_px(thickness_m=WALL_THICKNESS_M):
    return max(1, int(round(thickness_m / RESOLUTION)))


# ---------------------------------------------------------------------------
# Drawing primitives
# ---------------------------------------------------------------------------

def draw_wall(draw, x1, y1, x2, y2, thickness_m=WALL_THICKNESS_M):
    """Draw a straight wall segment from (x1,y1) to (x2,y2) in map metres."""
    p1 = m_to_px(x1, y1)
    p2 = m_to_px(x2, y2)
    draw.line([p1, p2], fill=COL_OCCUPIED, width=thickness_px(thickness_m))


def draw_rect(draw, x_min, y_min, x_max, y_max, thickness_m=WALL_THICKNESS_M):
    """Draw a rectangular obstacle (e.g. a pillar or piece of furniture)."""
    draw_wall(draw, x_min, y_min, x_max, y_min, thickness_m)
    draw_wall(draw, x_max, y_min, x_max, y_max, thickness_m)
    draw_wall(draw, x_max, y_max, x_min, y_max, thickness_m)
    draw_wall(draw, x_min, y_max, x_min, y_min, thickness_m)


def draw_filled_rect(draw, x_min, y_min, x_max, y_max):
    """Draw a solid filled rectangle (e.g. thick wall block or solid furniture)."""
    c1 = m_to_px(x_min, y_min)
    c2 = m_to_px(x_max, y_max)
    # Pillow rectangle wants (left, top, right, bottom)
    left   = min(c1[0], c2[0])
    top    = min(c1[1], c2[1])
    right  = max(c1[0], c2[0])
    bottom = max(c1[1], c2[1])
    draw.rectangle([left, top, right, bottom], fill=COL_OCCUPIED)


# ---------------------------------------------------------------------------
# ███████████████████████████████████████████████████████████████████████████
#
#   YOUR SURVEYED GEOMETRY GOES HERE
#
#   Replace the example below with your actual measurements.
#   Work room by room; add a comment for each wall so it's easy to update.
#
#   Tips:
#   - Measure inside faces of walls (what the lidar sees)
#   - Start from a known anchor (e.g. the desk corner at ~(1.66, -0.22))
#   - Door openings are simply gaps — don't draw anything there
#   - Run the script after each room to check the overlay
#
# ███████████████████████████████████████████████████████████████████████████

def draw_surveyed_map(draw):
    """
    Define all wall segments here.
    All coordinates in metres, map frame (origin = robot home position).

    EXAMPLE PLACEHOLDER — replace with your actual measurements.
    """

    # ------------------------------------------------------------------
    # OFFICE / HOME AREA
    # (robot home is at 0,0; desk corner at approx -0.2, +0.025)
    # N,S,E,W are map directions: N at top, E at right
    # Wall lines are centerlines (line width accounts for wall surface)
    # Measurement granularity = 0.005
    # ------------------------------------------------------------------

    # South wall of office
    # draw_wall(draw,  x1,   y1,   x2,   y2)
    draw_wall(draw, -1.51, -1.7,  1.585, -1.7)

    # North wall with (0.89 + 0.1) doorway gap
    draw_wall(draw, -1.51,  2.615, -1.21,  2.615)
    draw_wall(draw, -0.22,  2.615,  1.585,  2.615)

    # West wall
    draw_wall(draw, -1.51, -1.7, -1.51,  -1.22)
    draw_wall(draw, -1.585, -1.22, -1.585,  2.315)  # glass door (masked)
    draw_wall(draw, -1.51, 2.315, -1.51,  2.615)

    # East wall with (1.815 + 0.1)doorway gap
    draw_wall(draw,  1.585, -1.7,  1.585,  -0.5)   # south of doorway
    draw_wall(draw,  1.585,  1.415,  1.585,  2.615)   # north of doorway

    # Desk
    # draw_filled_rect(draw, x_min, y_min, x_max, y_max)
    # Right side
    draw_filled_rect(draw, -0.71, -0.365, -0.25, -0.025)
    # Left side
    draw_filled_rect(draw, -0.71, -1.46, -0.25, -1.12)
    # Back
    draw_wall(draw, -0.71, -0.25,  -0.71, -1.46)

    # File cabinet
    draw_filled_rect(draw, 1.0, -1.65, 1.53, -0.98)

    # Waste basket
    draw_filled_rect(draw, 0.8, -1.65, 0.95, -1.38)

    # ------------------------------------------------------------------
    # Dinette
    # (8.13 + 0.1) to north wall of house from south wall of office
    # ------------------------------------------------------------------

    # "Stub" walls on west
    draw_wall(draw,  -1.51,  6.2,  -1.51,  6.53)   # north of bay (meas=0.33)
    draw_wall(draw,  -1.51, 2.615,  -1.51,  2.965)   # south of bay (meas=0.35)

    # Bay offset walls (0.15 deep)
    draw_wall(draw,  -1.51,  6.2,  -1.66,  6.2)   # north
    draw_wall(draw,  -1.51, 2.965,  -1.66,  2.965)   # south

    # Bay (0.96 deep x 1.58 wide)
    draw_wall(draw,  -2.47,  5.37,  -1.66,  6.2)  # north angled section
    draw_wall(draw,  -2.47, 3.79,  -2.47,  5.37)  # center section (1.58 lg)
    draw_wall(draw,  -1.66, 2.965, -2.47,  3.79)  # south angled section

    # North wall of dinette (2.375 + 0.1) long
    draw_wall(draw, -1.51, 6.53,  0.965, 6.53)

    # ------------------------------------------------------------------
    # Kitchen
    # ------------------------------------------------------------------

    # Fridge wall
    draw_wall(draw, 2.877, 2.615,  2.877, 3.33)

    # Cabinets on south side
    draw_wall(draw, -0.146, 2.615,  -0.146, 2.825)
    draw_wall(draw, -0.146, 2.825,  0.20, 3.165)  # 45 degree
    draw_wall(draw, 0.20, 3.165,  0.97, 3.165)
    draw_wall(draw, 0.97, 3.25,  1.78, 3.25)
    draw_wall(draw, 1.78, 3.25,  1.78, 2.615)

    # Cabinets on north side
    draw_wall(draw, 0.965, 6.53,  0.965, 4.42)
    draw_wall(draw, 0.965, 4.42,  1.705, 4.42)
    draw_wall(draw, 1.705, 4.42,  1.705, 5.88)
    draw_wall(draw, 1.705, 5.88,  3.755, 5.88)
    draw_wall(draw, 3.755, 5.88,  3.755, 4.20)
    draw_wall(draw, 3.755, 4.20,  4.415, 4.20)

    # Fridge
    draw_filled_rect(draw, 1.915, 2.715, 2.735, 3.315)

    # ------------------------------------------------------------------
    # Piano room
    # ------------------------------------------------------------------

    # North wall
    draw_wall(draw, 4.415, 6.53,  8.685, 6.53)

    # West wall with doorway gap
    draw_wall(draw, 4.415, 2.615,  4.415, 2.75)
    draw_wall(draw, 4.415, 4.05,  4.415, 6.53)

    # East wall
    draw_wall(draw, 8.685, 6.53,  8.685, 3.07)

    # South "recess" wall
    draw_wall(draw, 8.685, 3.07,  7.78, 3.07)

    # ------------------------------------------------------------------
    # Living room
    # 10.243 from south wall to north wall of house
    # 8.203 from south end of east wall to north wall of house
    # north wall of foyer almost exactly aligned w/ south wall of office
    # ------------------------------------------------------------------

    # North wall
    draw_wall(draw, 1.585, 2.615,  4.415, 2.615)

    # East Wall
    draw_wall(draw, 7.78, 3.07,  7.78, -1.675)

    # South wall
    draw_wall(draw, 4.57, -3.81,  8.295, -3.81)

    # Guest suite doorway column (R)
    draw_wall(draw, 7.995, -3.81,  7.995, -3.51)
    draw_wall(draw, 7.995, -3.51,  8.225, -3.51)
    draw_wall(draw, 8.225, -3.51,  8.225, -3.81)

    # Oblique wall with doorway gap
    draw_wall(draw, 1.63, -1.7,  1.89, -1.89)
    draw_wall(draw, 2.625, -2.415,  4.57, -3.81)

    # MBR doorway column (R)
    draw_wall(draw, 1.585, -1.435,  1.82, -1.435)
    draw_wall(draw, 1.82, -1.435,  2.01, -1.57)
    draw_wall(draw, 2.01, -1.57,  1.82, -1.835)

    # MBR doorway column (L)
    draw_wall(draw, 2.695, -2.465,  2.855, -2.2)
    draw_wall(draw, 2.855, -2.2,  3.075, -2.335)
    draw_wall(draw, 3.075, -2.335,  3.075, -2.735)

    # TV table
    draw_filled_rect(draw, 7.28, -0.355, 7.58, 0.825)

    # Grandfather Clock
    draw_filled_rect(draw, 5.115, -3.71, 5.515, -3.46)

    # ------------------------------------------------------------------
    # Foyer
    # ------------------------------------------------------------------

    # North wall
    draw_wall(draw, 7.78, -1.675,  10.88, -1.675)

    # East Wall
    draw_wall(draw, 10.88, -1.675,  10.88, -3.81)

    # South wall beyond guest suite
    draw_wall(draw, 9.295, -3.81,  10.88, -3.81)
    
    # Guest suite doorway column (L)
    draw_wall(draw, 9.365, -3.81,  9.365, -3.51)
    draw_wall(draw, 9.365, -3.51,  9.585, -3.51)
    draw_wall(draw, 9.595, -3.51,  9.595, -3.81)

    # ------------------------------------------------------------------
    # Master Bedroom
    # ------------------------------------------------------------------

    # West wall
    draw_wall(draw, -1.55, -1.7, -1.55,  -6.055)

    # South wall
    draw_wall(draw, -1.55,  -6.055,  2.885, -6.055)

    # East Wall
    draw_wall(draw, 2.885, -6.055,  2.885, -2.6)


# ---------------------------------------------------------------------------
# Map generation
# ---------------------------------------------------------------------------

def build_map():
    # Start with all-white (free space)
    img = Image.new("L", (WIDTH_PX, HEIGHT_PX), color=COL_FREE)
    draw = ImageDraw.Draw(img)

    draw_surveyed_map(draw)

    # Save survey map
    img.save("survey_map.png")
    print(f"Saved survey_map.png  ({WIDTH_PX}×{HEIGHT_PX} px, "
          f"{WIDTH_PX*RESOLUTION:.1f}×{HEIGHT_PX*RESOLUTION:.1f} m)")

    # -------------------------------------------------------------------
    # Overlay: blend survey (red) over SLAM map (grey) for comparison
    # -------------------------------------------------------------------
    slam_file = "map_clean.png"
    try:
        slam = Image.open(slam_file).convert("L")
        if slam.size != (WIDTH_PX, HEIGHT_PX):
            slam = slam.resize((WIDTH_PX, HEIGHT_PX), Image.NEAREST)

        # RGB composite: SLAM map in greyscale, survey walls in red
        rgb_slam   = slam.convert("RGB")
        rgb_survey = img.convert("RGB")

        overlay = Image.new("RGB", (WIDTH_PX, HEIGHT_PX))
        slam_arr   = np.array(rgb_slam,   dtype=np.float32)
        survey_arr = np.array(rgb_survey, dtype=np.float32)

        # Where survey has a wall (dark pixel), tint red
        wall_mask = np.array(img) < 128          # True where survey wall
        composite = slam_arr.copy()
        composite[wall_mask] = [200, 50, 50]     # red survey walls

        # Mark home position (0,0) as a blue cross
        hx, hy = m_to_px(0.0, 0.0)
        for d in range(-5, 6):
            for col, row in [(hx+d, hy), (hx, hy+d)]:
                if 0 <= col < WIDTH_PX and 0 <= row < HEIGHT_PX:
                    composite[row, col] = [50, 50, 255]

        Image.fromarray(composite.astype(np.uint8)).save("survey_map_overlay.png")
        print("Saved survey_map_overlay.png  (red=survey walls, grey=SLAM map, blue=home)")

    except FileNotFoundError:
        print(f"Note: '{slam_file}' not found — overlay skipped")
    except Exception as e:
        print(f"Overlay error: {e}")


# ---------------------------------------------------------------------------
# Also regenerate the planning map (inflated obstacles for path planning)
# ---------------------------------------------------------------------------

def build_planning_map(robot_radius_m=0.20):
    """
    Inflate obstacles in survey_map.png by robot_radius to produce
    survey_planning.png — the map used by the path planner.
    """
    from scipy.ndimage import binary_dilation
    import struct

    try:
        img = Image.open("survey_map.png").convert("L")
    except FileNotFoundError:
        print("Run build_map() first to create survey_map.png")
        return

    arr = np.array(img)
    occupied = arr < 128

    radius_px = int(math.ceil(robot_radius_m / RESOLUTION))
    # Circular structuring element
    y_idx, x_idx = np.ogrid[-radius_px:radius_px+1, -radius_px:radius_px+1]
    struct_elem = (x_idx**2 + y_idx**2) <= radius_px**2

    inflated = binary_dilation(occupied, structure=struct_elem)
    result = np.where(inflated, COL_OCCUPIED, COL_FREE).astype(np.uint8)
    Image.fromarray(result).save("survey_planning.png")
    print(f"Saved survey_planning.png  (obstacles inflated by {robot_radius_m}m)")


# ---------------------------------------------------------------------------
# Convenience: print pixel coordinates for a map-frame point
# (useful when cross-referencing with your SLAM map in an image viewer)
# ---------------------------------------------------------------------------

def point_info(x_m, y_m, label=""):
    col, row = m_to_px(x_m, y_m)
    print(f"{label or f'({x_m},{y_m})':30s}  →  pixel col={col:4d}  row={row:4d}")


if __name__ == "__main__":
    # Print a few reference points so you can cross-check in your image viewer
    print("Reference points (map frame → pixel):")
    point_info(0.0,  0.0,  "Home position (0, 0)")
    point_info(-0.2, 0.025, "Desk corner (-0.2, +0.025)")
    print()

    build_map()
    build_planning_map()
