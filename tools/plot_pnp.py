#!/usr/bin/env python3
"""Plot the PnP recognition curve test results (CSV from pnp_curve_test).

Reads the CSV produced by build/pnp_curve_test and saves PNG plots:
  - camera/gimbal/world coordinates of the best armor over frames
  - distance over frames
  - Kalman-filtered world position vs raw world position
  - detected box center / size over frames

Usage:
  python3 tools/plot_pnp.py [out.csv] [out_dir]
Defaults: /tmp/pnp_curve.csv -> /tmp/pnp_curve_plots/
"""
import sys
import os
import csv
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append({k: float(v) if v not in ("",) else 0.0 for k, v in row.items()})
    return rows


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/pnp_curve.csv"
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/pnp_curve_plots"
    os.makedirs(out_dir, exist_ok=True)

    rows = load_csv(csv_path)
    if not rows:
        print("no data")
        return

    frames = np.arange(len(rows))
    t = np.array([r["t_ns"] for r in rows])
    if t[-1] > t[0]:
        t = (t - t[0]) * 1e-9  # seconds
    else:
        t = frames

    cam = np.array([[r["best_cam_x"], r["best_cam_y"], r["best_cam_z"]] for r in rows])
    g = np.array([[r["best_g_x"], r["best_g_y"], r["best_g_z"]] for r in rows])
    w = np.array([[r["best_w_x"], r["best_w_y"], r["best_w_z"]] for r in rows])
    kf = np.array([[r["kf_x"], r["kf_y"], r["kf_z"]] for r in rows])
    dist = np.array([r["best_dist"] for r in rows])
    narm = np.array([r["n_armors"] for r in rows])
    boxc = np.array([[r["box_cx"], r["box_cy"]] for r in rows])
    boxs = np.array([[r["box_w"], r["box_h"]] for r in rows])

    labels = ["x", "y", "z"]

    # 1) camera / gimbal / world coordinates
    for name, data in (("camera", cam), ("gimbal", g), ("world", w)):
        fig, ax = plt.subplots(figsize=(10, 4))
        for i in range(3):
            ax.plot(t, data[:, i], label=f"{name}_{labels[i]}",
                    color=["r", "g", "b"][i])
        ax.set_xlabel("time (s)")
        ax.set_ylabel("position (m)")
        ax.set_title(f"PnP {name} coordinates")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"{name}.png"), dpi=150)
        plt.close(fig)

    # 2) raw world vs Kalman world
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, w[:, i], "b-", alpha=0.5, label=f"raw {labels[i]}")
        if np.any(kf[:, i] != 0):
            ax.plot(t, kf[:, i], "r-", label=f"kf {labels[i]}")
        ax.set_ylabel(labels[i] + " (m)")
        ax.grid(True)
        ax.legend(loc="upper right")
    axes[0].set_title("World position: raw vs Kalman")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "world_raw_vs_kf.png"), dpi=150)
    plt.close(fig)

    # 3) distance
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, dist, "g-")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (m)")
    ax.set_title("PnP distance to best armor")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "distance.png"), dpi=150)
    plt.close(fig)

    # 4) detected box center + size
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    axes[0].plot(t, boxc[:, 0], "b-", label="box cx")
    axes[0].plot(t, boxc[:, 1], "r-", label="box cy")
    axes[0].set_ylabel("pixel")
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(t, boxs[:, 0], "b-", label="box w")
    axes[1].plot(t, boxs[:, 1], "r-", label="box h")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("pixel")
    axes[1].grid(True)
    axes[1].legend()
    axes[0].set_title("Detected armor box (pixels)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "box.png"), dpi=150)
    plt.close(fig)

    # 5) number of armors per frame
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(t, narm, "k.-")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("armors")
    ax.set_title("Armors detected per frame")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "armors.png"), dpi=150)
    plt.close(fig)

    print(f"saved plots to {out_dir}")


if __name__ == "__main__":
    main()
