# build_map.py
"""
Surveyed map builder.

Produces three PNG map files matching map_metadata.json exactly:

    survey_map.png          — fixed obstacles only (walls + immovable furniture)
                              Used for ICP localization.

    survey_planning_base.png— fixed + moveable obstacles combined
                              (before inflation)

    survey_planning.png     — planning_base inflated by robot radius
                              Used by the path planner.

    survey_map_overlay.png  — survey_map (red) over SLAM map (grey)
                              Used to verify survey accuracy.

Coordinate system
-----------------
All coordinates are in METRES in the robot's map frame:
    +x = east  (forward from home position — robot faces east at startup)
    +y = north  (leftward from home position)
    Origin (0, 0) = robot home position

Workflow
--------
1. Add/edit geometry in draw_fixed_features() and draw_moveable_obstacles().
2. Run:  python build_map.py
3. Inspect survey_map_overlay.png to verify alignment with SLAM map.
4. Copy survey_map.png     → icp_localizer.py  (MAP_FILE)
   Copy survey_planning.png → path planner      (planning map)

Wall drawing convention
-----------------------
- draw_wall() coordinates are CENTERLINES; line width covers the wall surface.
- draw_filled_rect() coordinates are the OBSTACLE SURFACE extents.
- WALL_THICKNESS_M = 0.10 m default (2 px total = 1 px either side of CL).
- Doors are gaps — simply omit those segments.
- Oblique obstacles: use CAD to derive endpoint coordinates, then enter here.
"""

import math
import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_dilation

# ---------------------------------------------------------------------------
# Map parameters — must match map_metadata.json exactly
# ---------------------------------------------------------------------------
RESOLUTION  = 0.05    # metres per pixel
WIDTH_PX    = 300     # image width  in pixels
HEIGHT_PX   = 300     # image height in pixels
ORIGIN_X    = -3.0    # map-frame x of image left  edge
ORIGIN_Y    = -7.0    # map-frame y of image bottom edge

# ---------------------------------------------------------------------------
# Drawing defaults
# ---------------------------------------------------------------------------
WALL_THICKNESS_M = 0.10   # metres — drawn symmetrically about the centreline

# Pixel values
COL_FREE     = 255
COL_OCCUPIED = 0

# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def m_to_px(x_m, y_m):
    """Map-frame metres → Pillow pixel (col, row).  Row 0 = top of image."""
    col = int((x_m - ORIGIN_X) / RESOLUTION)
    row = HEIGHT_PX - 1 - int((y_m - ORIGIN_Y) / RESOLUTION)
    return col, row


def thickness_px(thickness_m=WALL_THICKNESS_M):
    return max(1, int(round(thickness_m / RESOLUTION)))


# ---------------------------------------------------------------------------
# Drawing primitives
# ---------------------------------------------------------------------------

def draw_wall(draw, x1, y1, x2, y2, thickness_m=WALL_THICKNESS_M):
    """Straight wall segment, centreline from (x1,y1) to (x2,y2)."""
    draw.line([m_to_px(x1, y1), m_to_px(x2, y2)],
              fill=COL_OCCUPIED, width=thickness_px(thickness_m))


def draw_rect(draw, x_min, y_min, x_max, y_max, thickness_m=WALL_THICKNESS_M):
    """Hollow rectangle outline — four wall segments."""
    draw_wall(draw, x_min, y_min, x_max, y_min, thickness_m)
    draw_wall(draw, x_max, y_min, x_max, y_max, thickness_m)
    draw_wall(draw, x_max, y_max, x_min, y_max, thickness_m)
    draw_wall(draw, x_min, y_max, x_min, y_min, thickness_m)


def draw_filled_rect(draw, x_min, y_min, x_max, y_max):
    """Solid filled rectangle — use for compact immovable obstacles."""
    c1 = m_to_px(x_min, y_min)
    c2 = m_to_px(x_max, y_max)
    left   = min(c1[0], c2[0])
    top    = min(c1[1], c2[1])
    right  = max(c1[0], c2[0])
    bottom = max(c1[1], c2[1])
    draw.rectangle([left, top, right, bottom], fill=COL_OCCUPIED)


# ===========================================================================
#
#   GEOMETRY — edit these two functions to define your environment
#
# ===========================================================================

def draw_fixed_features(draw):
    """
    Structural walls and immovable furniture.

    Drawn on BOTH the localization map and the planning map.
    These are the features ICP trusts as ground truth.

    N,S,E,W = map compass directions (N at top of image).
    All coordinates in metres, map frame.
    Measurement granularity = 0.005 m.
    """

    # ------------------------------------------------------------------
    # OFFICE / HOME AREA
    # Robot home at (0,0); desk corner at approx (-0.2, +0.025)
    # ------------------------------------------------------------------

    # South wall
    draw_wall(draw, -1.51, -1.7,   1.585, -1.7)

    # North wall — doorway gap (0.89 + 0.1 m)
    draw_wall(draw, -1.51,  2.615, -1.21,  2.615)
    draw_wall(draw, -0.22,  2.615,  1.585,  2.615)

    # West wall — glass door section recessed 0.075 m
    draw_wall(draw, -1.51,  -1.7,  -1.51,  -1.22)
    draw_wall(draw, -1.585, -1.22, -1.585,  2.315)   # glass door (masked)
    draw_wall(draw, -1.51,   2.315, -1.51,   2.615)

    # East wall — doorway gap (1.815 + 0.1 m)
    draw_wall(draw,  1.585, -1.7,   1.585, -0.5)
    draw_wall(draw,  1.585,  1.415,  1.585,  2.615)

    # Desk (L-shaped — two filled pads + back wall)
    draw_filled_rect(draw, -0.71, -0.365, -0.25, -0.025)  # right side
    draw_filled_rect(draw, -0.71, -1.46,  -0.25, -1.12)   # left side
    draw_wall(draw,        -0.71, -0.25,  -0.71, -1.46)   # back

    # File cabinet
    draw_filled_rect(draw, 1.0,  -1.65,  1.53, -0.98)

    # Waste basket
    draw_filled_rect(draw, 0.8,  -1.65,  0.95, -1.38)

    # Glass top table
    draw_filled_rect(draw, 0.85,  1.835,  1.105, 2.135)

    # Temporary ottoman
    # draw_filled_rect(draw, 0.312, 0.207, 1.168, 0.923)  # Centre: (0.740, 0.565)

    # ------------------------------------------------------------------
    # DINETTE
    # 8.13 + 0.1 m from south wall of office to north wall of house
    # ------------------------------------------------------------------

    # West stub walls (either side of bay)
    draw_wall(draw, -1.51,  6.2,   -1.51,  6.53)    # north of bay
    draw_wall(draw, -1.51,  2.615, -1.51,  2.965)   # south of bay

    # Bay offset walls (0.15 m deep)
    draw_wall(draw, -1.51,  6.2,   -1.66,  6.2)
    draw_wall(draw, -1.51,  2.965, -1.66,  2.965)

    # Bay walls (0.96 deep × 1.58 wide)
    draw_wall(draw, -2.47,  5.37,  -1.66,  6.2)     # north angled
    draw_wall(draw, -2.47,  3.79,  -2.47,  5.37)    # centre
    draw_wall(draw, -1.66,  2.965, -2.47,  3.79)    # south angled

    # North wall of dinette
    draw_wall(draw, -1.51,  6.53,   0.965,  6.53)

    # ------------------------------------------------------------------
    # KITCHEN
    # ------------------------------------------------------------------

    # Fridge wall (between fridge and living room doorway)
    draw_wall(draw, 2.877,  2.615,  2.877,  3.33)

    # Cabinets — south side
    draw_wall(draw, -0.146, 2.615, -0.146,  2.825)
    draw_wall(draw, -0.146, 2.825,  0.20,   3.165)  # 45°
    draw_wall(draw,  0.20,  3.165,  0.97,   3.165)
    draw_wall(draw,  0.97,  3.25,   1.78,   3.25)
    draw_wall(draw,  1.78,  3.25,   1.78,   2.615)

    # Cabinets — north side
    draw_wall(draw,  0.965, 6.53,   0.965,  4.42)
    draw_wall(draw,  0.965, 4.42,   1.515,  4.42)
    draw_wall(draw,  1.515, 4.42,   1.515,  5.88)
    draw_wall(draw,  1.515, 5.88,   3.765,  5.88)
    draw_wall(draw,  3.765, 5.88,   3.765,  4.20)
    draw_wall(draw,  3.765, 4.20,   4.415,  4.20)

    # Fridge
    draw_filled_rect(draw, 1.915, 2.715,  2.735,  3.315)

    # ------------------------------------------------------------------
    # PIANO ROOM
    # ------------------------------------------------------------------

    # North wall
    draw_wall(draw, 4.415,  6.53,   8.685,  6.53)

    # West wall — doorway gap
    draw_wall(draw, 4.415,  2.615,  4.415,  2.75)
    draw_wall(draw, 4.415,  4.05,   4.415,  6.53)

    # East wall
    draw_wall(draw, 8.685,  6.53,   8.685,  3.07)

    # South recess wall
    draw_wall(draw, 8.685,  3.07,   7.78,   3.07)

    # Table East wall
    #draw_filled_rect(draw, 4.600, 4.852, 5.045, 5.436)

    # Corner book case
    draw_wall(draw, 8.245, 3.19,  8.035, 3.4)
    draw_wall(draw, 8.035, 3.4, 8.355, 3.72)
    draw_wall(draw, 8.355, 3.72,  8.565, 3.51)
    draw_wall(draw, 8.565, 3.51,  8.245, 3.19)

    # Piano
    draw_filled_rect(draw, 6.634, 5.620,  6.684, 5.670)  # R leg (6.659, 5.645)
    draw_filled_rect(draw, 7.121, 4.419, 7.171, 4.469)  # L leg (7.146, 4.444)
    draw_filled_rect(draw, 5.515, 4.08, 5.565, 4.13)  # back leg (5.54, 4.105)
    draw_filled_rect(draw, 6.801, 5.069,  6.851, 5.119)  # pedals R (6.826, 5.094)
    draw_filled_rect(draw, 6.856, 4.936,  6.906, 4.986)  # pedals L (6.881, 4.961)
    
    # Bench
    draw_filled_rect(draw, 7.458, 5.478,  7.508, 5.528)  # RF leg (7.483, 5.503)
    draw_filled_rect(draw, 7.615, 5.064,  7.665, 5.114)  # LF leg (7.64, 5.089)
    draw_filled_rect(draw, 7.904, 5.170,  7.954, 5.220)  # LR leg (7.929, 5.195)
    draw_filled_rect(draw, 7.750, 5.586,  7.800, 5.636)  # RR leg (7.775, 5.611)

    # ------------------------------------------------------------------
    # LIVING ROOM
    # ------------------------------------------------------------------

    # North wall
    draw_wall(draw, 1.585,  2.615,  4.415,  2.615)

    # East wall
    draw_wall(draw, 7.78,   3.07,   7.78,  -1.675)

    # South wall
    draw_wall(draw, 4.57,  -3.81,   8.295,  -3.81)

    # Oblique south-west wall — doorway gap
    draw_wall(draw, 1.63,  -1.7,    1.89,  -1.89)
    draw_wall(draw, 2.625, -2.415,  4.57,  -3.81)

    # MBR doorway column — right
    draw_wall(draw, 1.585, -1.435,  1.82,  -1.435)
    draw_wall(draw, 1.82,  -1.435,  2.01,  -1.57)
    draw_wall(draw, 2.01,  -1.57,   1.82,  -1.835)

    # MBR doorway column — left
    draw_wall(draw, 2.695, -2.465,  2.855, -2.2)
    draw_wall(draw, 2.855, -2.2,    3.075, -2.335)
    draw_wall(draw, 3.075, -2.335,  3.075, -2.735)

    # TV table
    draw_filled_rect(draw, 7.28,  -0.355,  7.58,   0.825)

    # Grandfather clock
    draw_filled_rect(draw, 5.115, -3.71,   5.515, -3.46)

    # ------------------------------------------------------------------
    # FOYER
    # ------------------------------------------------------------------

    # North wall
    draw_wall(draw, 7.78,  -1.675, 8.9,  -1.675)
    draw_wall(draw, 9.82,  -1.675, 10.88,  -1.675)

    # East wall
    draw_wall(draw, 10.88, -1.675, 10.88,  -3.81)

    # South wall
    draw_wall(draw, 9.295, -3.81,   10.88,  -3.81)

    # Doorway column L
    draw_filled_rect(draw, 7.995, -3.8,   8.225, -3.61)

    # Doorway column R
    draw_filled_rect(draw, 9.365, -3.8,   9.595, -3.61)

    # ------------------------------------------------------------------
    # MASTER BEDROOM
    # ------------------------------------------------------------------

    # West wall
    draw_wall(draw, -1.55, -1.7,   -1.55,  -6.055)

    # South wall
    draw_wall(draw, -1.55, -6.055,  1.05, -6.055)
    draw_wall(draw, 1.95, -6.055,  2.15, -6.055)

    # Oblique SE corner
    draw_wall(draw, 2.15, -6.055,  2.885, -5.4)

    # East wall
    draw_wall(draw,  2.885, -5.4,  2.885, -2.6)

    # King bed
    draw_filled_rect(draw, -1.51, -4.835,  0.675, -2.915)

    # Night table R
    draw_filled_rect(draw, -1.4, -2.7,  -1.05, -1.9)

    # Night table L
    draw_filled_rect(draw, -1.4, -5.9,  -1.05, -5.1)

    # Dresser
    draw_filled_rect(draw, 2.405, -4.815,  2.785, -3.315)

    # Chest
    draw_wall(draw, 2.20, -5.78,  2.03, -5.555)
    draw_wall(draw, 2.03, -5.555,  2.43, -5.15)
    draw_wall(draw, 2.43, -5.15,  2.65, -5.43)
    draw_wall(draw, 2.65, -5.43,  2.20, -5.78)

def draw_moveable_obstacles(draw):
    """
    Approximate positions of semi-permanent moveable furniture.

    Drawn ONLY on the planning map — NOT used for localization.
    Coordinates are conservative bounding boxes (slightly oversized is safe).
    Precise surveying is not required; rough measurements are fine.

    Add furniture here as you measure it.
    """

    # Sofa — living room
    draw_filled_rect(draw, 2.965, -0.665, 3.775, 1.265)

    # Armchair(s)
    draw_filled_rect(draw, 4.98, -1.645,  5.66, -0.965)

    # Coffee table
    draw_filled_rect(draw, 4.375, -0.145,  4.755, 0.755)

    # Swivel chair
    draw_wall(draw, 4.75, 2,  5.1, 1.55)
    draw_wall(draw, 5.1, 1.55,  5.5, 1.9)
    draw_wall(draw, 5.5, 1.9,  5.15, 2.35)
    draw_wall(draw, 5.15, 2.35,  4.75, 2)

    # Ottoman
    draw_wall(draw, 5.45, 1.0,  5.9, 1.35)
    draw_wall(draw, 5.9, 1.35,  6.1, 1.0)
    draw_wall(draw, 6.1, 1.0,  5.7, 0.65)
    draw_wall(draw, 5.7, 0.65,  5.45, 1.0)

    # Basket
    draw_filled_rect(draw, 7.3, 1.72,  7.7, 2.17)

    # Kitchen table & chairs
    draw_filled_rect(draw, -1.265, 3.93,  -0.435, 5.33)
    draw_filled_rect(draw, -1.05, 3.73,  -0.65, 5.53)
    draw_filled_rect(draw, -1.465, 4.43,  -0.235, 4.83)

# ===========================================================================
#   Map generation — no need to edit below here
# ===========================================================================

def _new_canvas():
    return Image.new("L", (WIDTH_PX, HEIGHT_PX), color=COL_FREE)


def build_localization_map():
    """Fixed features only → survey_map.png  (used by ICP localizer)."""
    img = _new_canvas()
    draw_fixed_features(ImageDraw.Draw(img))
    img.save("survey_map.png")
    print(f"Saved survey_map.png        "
          f"({WIDTH_PX}×{HEIGHT_PX} px = "
          f"{WIDTH_PX*RESOLUTION:.1f}×{HEIGHT_PX*RESOLUTION:.1f} m)")
    return img


def build_planning_map(robot_radius_m=0.20):
    """Fixed + moveable features, then inflated → survey_planning.png."""
    # --- base: fixed + moveable ---
    img = _new_canvas()
    d = ImageDraw.Draw(img)
    draw_fixed_features(d)
    draw_moveable_obstacles(d)
    img.save("survey_planning_base.png")
    print(f"Saved survey_planning_base.png  (fixed + moveable, pre-inflation)")

    # --- inflate by robot radius ---
    arr      = np.array(img)
    occupied = arr < 128
    r_px     = int(math.ceil(robot_radius_m / RESOLUTION))
    yi, xi   = np.ogrid[-r_px:r_px+1, -r_px:r_px+1]
    struct   = (xi**2 + yi**2) <= r_px**2
    inflated = binary_dilation(occupied, structure=struct)
    result   = np.where(inflated, COL_OCCUPIED, COL_FREE).astype(np.uint8)
    Image.fromarray(result).save("survey_planning.png")
    print(f"Saved survey_planning.png       "
          f"(inflated by robot radius {robot_radius_m} m)")


def build_overlay(loc_img):
    """Overlay survey walls (red) on SLAM map (grey) → survey_map_overlay.png."""
    slam_file = "map_clean.png"
    try:
        slam = Image.open(slam_file).convert("L")
        if slam.size != (WIDTH_PX, HEIGHT_PX):
            slam = slam.resize((WIDTH_PX, HEIGHT_PX), Image.NEAREST)

        composite = np.array(slam.convert("RGB"), dtype=np.float32)

        # Survey walls → red
        wall_mask = np.array(loc_img) < 128
        composite[wall_mask] = [200, 50, 50]

        # Home position (0,0) → blue cross
        hx, hy = m_to_px(0.0, 0.0)
        for d in range(-5, 6):
            for col, row in [(hx+d, hy), (hx, hy+d)]:
                if 0 <= col < WIDTH_PX and 0 <= row < HEIGHT_PX:
                    composite[row, col] = [50, 50, 255]

        Image.fromarray(composite.astype(np.uint8)).save("survey_map_overlay.png")
        print("Saved survey_map_overlay.png    "
              "(red=survey walls, grey=SLAM map, blue+=home)")

    except FileNotFoundError:
        print(f"Note: '{slam_file}' not found — overlay skipped")
    except Exception as e:
        print(f"Overlay error: {e}")


def point_info(x_m, y_m, label=""):
    """Print the pixel location of a map-frame point — useful for verification."""
    col, row = m_to_px(x_m, y_m)
    print(f"  {label or f'({x_m}, {y_m})':35s} → col={col:4d}  row={row:4d}")


if __name__ == "__main__":
    print("Reference points:")
    point_info(0.0,   0.0,   "Home position (0, 0)")
    point_info(-0.2,  0.025, "Desk corner (-0.2, +0.025)")
    print()

    loc_img = build_localization_map()
    build_planning_map()
    build_overlay(loc_img)
