#!/usr/bin/env python3
"""Compare PnP recognition curves across multiple pnp_curve_test CSV recordings.

Usage:
  python3 tools/plot_pnp_compare.py [csv1 csv2 ...] [out_dir]
  (defaults: the three reference recordings under /tmp, output to /tmp/pnp_compare)

Each CSV is produced by build/pnp_curve_test and has one row per frame:
  seq, t_ns, n_armors, best_cam_xyz, best_g_xyz, best_w_xyz, best_dist,
  kf_xyz, box_cx/cy/w/h

Plots (saved to out_dir):
  - world_raw_compare.png       raw world x/y/z for each recording
  - raw_vs_kf_<name>.png        raw vs Kalman world position per recording
  - distance_compare.png        PnP distance over time, all recordings
  - cam_x_compare.png           camera-frame x (jitter indicator) per recording
"""
import sys
import os
import csv

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return None
    t = np.array([r["t_ns"] for r in rows], float)
    t = (t - t[0]) * 1e-9  # seconds
    w = np.array([[r["best_w_x"], r["best_w_y"], r["best_w_z"]] for r in rows], float)
    kf = np.array([[r["kf_x"], r["kf_y"], r["kf_z"]] for r in rows], float)
    dist = np.array([r["best_dist"] for r in rows], float)
    cam = np.array([[r["best_cam_x"], r["best_cam_y"], r["best_cam_z"]] for r in rows], float)
    return t, w, kf, dist, cam


def main():
    args = sys.argv[1:]
    out_dir = "/tmp/pnp_compare"
    if args and args[-1].endswith(".png") is False and not os.path.exists(args[-1]) \
            and len(args) > 1 and not os.path.isfile(args[-1]):
        pass  # handled below
    # Parse: trailing arg that is a directory -> out_dir
    files = args
    if len(files) >= 1 and not os.path.isfile(files[-1]) and not files[-1].endswith(".csv"):
        out_dir = files[-1]
        files = files[:-1]
    if not files:
        files = ["/tmp/pnp_curve.csv", "/tmp/pnp_curve_spin.csv",
                 "/tmp/pnp_curve_linear.csv"]
    os.makedirs(out_dir, exist_ok=True)

    data = []
    labels = []
    for f in files:
        if not os.path.isfile(f):
            print(f"skip missing: {f}")
            continue
        d = load(f)
        if d is not None:
            data.append(d)
            labels.append(os.path.basename(f).replace(".csv", ""))
    if not data:
        print("no data to plot")
        return

    # raw world x/y/z per recording
    fig, axes = plt.subplots(len(data), 1, figsize=(12, 3 * len(data)), sharex=True)
    if len(data) == 1:
        axes = [axes]
    for ax, (t, w, kf, dist, cam), lab in zip(axes, data, labels):
        for i in range(3):
            ax.plot(t, w[:, i], ".-", lw=0.6, ms=2, label=["x", "y", "z"][i])
        ax.set_ylabel("world (m)")
        ax.set_title(lab)
        ax.grid(True)
        ax.legend(loc="upper right", ncol=3)
    axes[0].set_title("Raw world position (stationary / spin / linear)")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "world_raw_compare.png"), dpi=150)
    plt.close(fig)

    # raw vs Kalman per recording
    for (t, w, kf, dist, cam), lab in zip(data, labels):
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        for i, ax in enumerate(axes):
            ax.plot(t, w[:, i], "b-", alpha=0.5, lw=1, label=f"raw {['x','y','z'][i]}")
            ax.plot(t, kf[:, i], "r-", lw=1, label=f"kf {['x','y','z'][i]}")
            ax.set_ylabel("(m)")
            ax.grid(True)
            ax.legend(loc="upper right")
        axes[0].set_title(f"Raw vs Kalman world position - {lab}")
        axes[-1].set_xlabel("time (s)")
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, f"raw_vs_kf_{lab}.png"), dpi=150)
        plt.close(fig)

    # distance comparison
    fig, ax = plt.subplots(figsize=(12, 4))
    for (t, w, kf, dist, cam), lab in zip(data, labels):
        ax.plot(t, dist, "-", lw=1, label=lab)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (m)")
    ax.set_title("PnP distance across recordings")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "distance_compare.png"), dpi=150)
    plt.close(fig)

    # cam_x jitter indicator
    fig, axes = plt.subplots(len(data), 1, figsize=(12, 3 * len(data)), sharex=True)
    if len(data) == 1:
        axes = [axes]
    for ax, (t, w, kf, dist, cam), lab in zip(axes, data, labels):
        ax.plot(t, cam[:, 0], ".-", lw=0.6, ms=2)
        ax.set_ylabel("cam_x (m)")
        ax.set_title(f"cam_x {lab}")
        ax.grid(True)
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "cam_x_compare.png"), dpi=150)
    plt.close(fig)

    print(f"saved {len(data)} recordings -> {out_dir}")


if __name__ == "__main__":
    main()
