# plot_icp_log.py
"""
Plot the ICP correction log produced by icp_localizer.py.

Usage:
    python plot_icp_log.py                    # auto-finds the most recent log
    python plot_icp_log.py icp_log_XXXX.csv  # specific file

Produces a 4-panel figure showing:
  1. Robot trajectory — odometry (blue) vs ICP-corrected map frame (red)
  2. Correction translation magnitude over time
  3. Correction rotation (dtheta) over time
  4. ICP mean correspondence error over time
"""

import sys
import csv
import glob
import math
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


# ---------------------------------------------------------------------------
# Load log
# ---------------------------------------------------------------------------

def load_log(filename):
    rows = []
    with open(filename) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    print(f"Loaded {len(rows)} cycles from '{filename}'")
    return rows


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot(rows, filename):
    t          = np.array([r["elapsed_s"]       for r in rows])
    odom_x     = np.array([r["odom_x"]          for r in rows])
    odom_y     = np.array([r["odom_y"]          for r in rows])
    map_x      = np.array([r["map_x"]           for r in rows])
    map_y      = np.array([r["map_y"]           for r in rows])
    corr_dx    = np.array([r["corr_dx"]         for r in rows])
    corr_dy    = np.array([r["corr_dy"]         for r in rows])
    corr_dth   = np.array([r["corr_dtheta_deg"] for r in rows])
    icp_err    = np.array([r["icp_err"]         for r in rows])

    # Filter out inf err values for plotting
    err_finite = np.where(np.isfinite(icp_err), icp_err, np.nan)

    corr_mag = np.sqrt(corr_dx**2 + corr_dy**2)

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(f"ICP Localizer — {os.path.basename(filename)}", fontsize=12)
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.3)

    # ------------------------------------------------------------------
    # Panel 1: Trajectory
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(odom_x, odom_y, "b-", linewidth=1.2, label="Odometry (raw)")
    ax1.plot(map_x,  map_y,  "r-", linewidth=1.2, label="ICP-corrected (map frame)")
    ax1.plot(odom_x[0], odom_y[0], "go", markersize=8, label="Start")
    ax1.plot(odom_x[-1], odom_y[-1], "rs", markersize=8, label="End")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Robot Trajectory")
    ax1.legend(fontsize=8)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)

    # Annotate drift: distance between start and end in odom frame
    odom_drift = math.sqrt((odom_x[-1]-odom_x[0])**2 + (odom_y[-1]-odom_y[0])**2)
    map_drift  = math.sqrt((map_x[-1]-map_x[0])**2   + (map_y[-1]-map_y[0])**2)
    ax1.text(0.02, 0.02,
             f"Odom return error: {odom_drift*100:.1f} cm\n"
             f"ICP return error:  {map_drift*100:.1f} cm",
             transform=ax1.transAxes, fontsize=8,
             verticalalignment="bottom",
             bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    # ------------------------------------------------------------------
    # Panel 2: Correction translation magnitude
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t, corr_mag * 100, "purple", linewidth=1.2)
    ax2.fill_between(t, 0, corr_mag * 100, alpha=0.15, color="purple")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Correction magnitude (cm)")
    ax2.set_title("Accumulated Translation Correction")
    ax2.grid(True, alpha=0.3)
    ax2.text(0.02, 0.97,
             f"Peak: {corr_mag.max()*100:.1f} cm\n"
             f"Final: {corr_mag[-1]*100:.1f} cm",
             transform=ax2.transAxes, fontsize=8,
             verticalalignment="top",
             bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    # ------------------------------------------------------------------
    # Panel 3: Correction rotation
    # ------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(t, corr_dth, "darkorange", linewidth=1.2)
    ax3.axhline(0, color="k", linewidth=0.5, linestyle="--")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Correction rotation (°)")
    ax3.set_title("Accumulated Heading Correction")
    ax3.grid(True, alpha=0.3)
    ax3.text(0.02, 0.97,
             f"Peak: {abs(corr_dth).max():.2f}°\n"
             f"Final: {corr_dth[-1]:.2f}°",
             transform=ax3.transAxes, fontsize=8,
             verticalalignment="top",
             bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    # ------------------------------------------------------------------
    # Panel 4: ICP mean error
    # ------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(t, err_finite * 100, "steelblue", linewidth=1.2)
    ax4.axhline(5.0, color="green",  linewidth=1, linestyle="--",
                label="Good threshold (5 cm)")
    ax4.axhline(10.0, color="orange", linewidth=1, linestyle="--",
                label="Marginal threshold (10 cm)")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("ICP mean error (cm)")
    ax4.set_title("ICP Correspondence Error")
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    finite = err_finite[np.isfinite(err_finite)]
    if len(finite):
        ax4.text(0.02, 0.97,
                 f"Mean: {finite.mean()*100:.1f} cm\n"
                 f"Peak: {finite.max()*100:.1f} cm",
                 transform=ax4.transAxes, fontsize=8,
                 verticalalignment="top",
                 bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    out = filename.replace(".csv", ".png")
    plt.savefig(out, dpi=150, bbox_inches="tight")
    print(f"Saved {out}")
    plt.show()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # Auto-find most recent log
        logs = sorted(glob.glob("icp_logs/icp_log_*.csv"))
        if not logs:
            print("No icp_log_*.csv files found")
            print("Usage: python plot_icp_log.py [logfile.csv]")
            sys.exit(1)
        filename = logs[-1]
        print(f"Auto-selected most recent log: {filename}")

    rows = load_log(filename)
    if not rows:
        print("Log file is empty.")
        sys.exit(1)

    plot(rows, filename)
