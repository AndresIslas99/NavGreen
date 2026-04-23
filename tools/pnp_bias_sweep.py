#!/usr/bin/env python3
"""PnP depth bias sweep at settle geometry.

Tests SQPNP at multiple base→tag standoffs around c1_approach goal (4.67):
robot at base.x = 4.30 .. 5.50, yaw=π, tag at (3.67, 0, 0.002, yaw=0),
cam at base + (0.7, 0.06, 0.010) world → optical, 672x376 fx=235.27.

Reports for each standoff:
  - GT range, GT tvec.z (=cam-to-tag forward in optical)
  - PnP mean tvec.z, bias (mean - GT)
  - σ_z, σ_z after median-15
"""
from __future__ import annotations
import math
import sys
import numpy as np
import cv2

# Camera intrinsics (read live earlier)
fx = fy = 235.26972949873274
cx = 336.0; cy = 188.0
W, H = 672, 376
TAG_SIZE = 0.20
half = TAG_SIZE / 2.0

K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
dist = np.zeros(4, dtype=np.float64)
obj = np.array([
    [-half, -half, 0.0],
    [ half, -half, 0.0],
    [ half,  half, 0.0],
    [-half,  half, 0.0],
], dtype=np.float64)


def rotz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def project(corners_world, cam_pos, R_world_cam):
    R_cam_world = R_world_cam.T
    rel = (corners_world - cam_pos)
    cam_pts = (R_cam_world @ rel.T).T
    pix = np.zeros((4, 2))
    for i in range(4):
        Xc, Yc, Zc = cam_pts[i]
        pix[i, 0] = fx * Xc / Zc + cx
        pix[i, 1] = fy * Yc / Zc + cy
    return pix, cam_pts


def gt_tvec_func(corners_world, cam_pos, R_world_cam):
    origin = corners_world.mean(axis=0)
    return R_world_cam.T @ (origin - cam_pos)


def run_at_standoff(base_x, base_y=0.0, base_yaw=math.pi, base_z=0.20,
                    cam_in_base=(0.70, 0.06, 0.010),
                    tag_xy=(3.67, 0.0), tag_z=0.002, tag_yaw=0.0,
                    n=2000, noise_px=1.0, seed=42):
    R_base_world = rotz(base_yaw)
    base_pos = np.array([base_x, base_y, base_z])
    cam_pos_world = base_pos + R_base_world @ np.array(cam_in_base)

    fwd = R_base_world @ np.array([1.0, 0.0, 0.0])
    left = R_base_world @ np.array([0.0, 1.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    R_world_cam = np.stack([-left, -up, fwd], axis=1)  # cam optical X=right, Y=down, Z=fwd

    # Tag corners
    R_world_tag = rotz(tag_yaw)
    centre = np.array([tag_xy[0], tag_xy[1], tag_z])
    corners_world = (R_world_tag @ obj.T).T + centre

    # GT
    pix_gt, cam_pts = project(corners_world, cam_pos_world, R_world_cam)
    gt_tvec = gt_tvec_func(corners_world, cam_pos_world, R_world_cam)
    in_frame = all(0 <= u <= W and 0 <= v <= H for u, v in pix_gt)
    span = max(pix_gt[:, 0].max() - pix_gt[:, 0].min(),
               pix_gt[:, 1].max() - pix_gt[:, 1].min())
    incidence = math.degrees(math.acos(min(1.0, abs(
        (cam_pos_world - corners_world.mean(0))[2] /
        np.linalg.norm(cam_pos_world - corners_world.mean(0))))))

    rng = np.random.default_rng(seed)
    z_meas = []
    x_meas = []
    failed = 0
    for _ in range(n):
        noise = rng.normal(0.0, noise_px, pix_gt.shape)
        pix_n = pix_gt + noise
        try:
            ok, rvec, tvec = cv2.solvePnP(
                obj, pix_n, K, dist, flags=cv2.SOLVEPNP_SQPNP)
            if not ok:
                failed += 1
                continue
            tvec = tvec.flatten()
            z_meas.append(tvec[2])
            x_meas.append(tvec[0])
        except cv2.error:
            failed += 1
    if not z_meas:
        return None
    z = np.array(z_meas)
    x = np.array(x_meas)

    # Median window 15
    def med15(arr):
        out = []
        for i in range(len(arr) - 14):
            out.append(np.median(arr[i:i+15]))
        return np.array(out)

    return {
        "gt_z": float(gt_tvec[2]),
        "gt_x": float(gt_tvec[0]),
        "gt_y_cam": float(gt_tvec[1]),
        "gt_range": float(np.linalg.norm(gt_tvec)),
        "incidence_deg": incidence,
        "tag_px_span": span,
        "in_frame": in_frame,
        "mean_z": float(z.mean()),
        "bias_z": float(z.mean() - gt_tvec[2]),
        "std_z": float(z.std()),
        "median15_mean_z": float(med15(z).mean()),
        "median15_bias_z": float(med15(z).mean() - gt_tvec[2]),
        "median15_std_z": float(med15(z).std()),
        "fail_pct": 100.0 * failed / n,
    }


def main():
    print("PnP depth bias sweep — settle geometry, SQPNP, 1px gaussian, N=2000")
    print("Tag at (3.67, 0, 0.002, yaw=0). Robot facing π. cam offset (+0.7, +0.06, +0.010)")
    print()
    print("%-9s %-9s %-7s %-8s %-7s %-9s %-9s %-9s %-12s %-13s" %
          ("base.x", "GT tvec.z", "incdeg", "px_span", "fails%",
           "mean.z", "σ_z", "bias_z", "med15_bias", "med15_σ_z"))
    print("-" * 120)
    # Sweep around the goal (4.67) and the actual settle pose (4.80)
    standoffs = [4.30, 4.40, 4.50, 4.60, 4.67, 4.70, 4.75, 4.80, 4.85, 4.90,
                 5.00, 5.20, 5.50]
    for base_x in standoffs:
        r = run_at_standoff(base_x)
        if r is None:
            print(f"{base_x:7.2f}  ALL FAILED")
            continue
        print("%-9.2f %-9.4f %-7.1f %-8.0f %-7.1f %-9.4f %-9.5f %-9.5f %-12.5f %-13.5f" %
              (base_x, r["gt_z"], r["incidence_deg"], r["tag_px_span"], r["fail_pct"],
               r["mean_z"], r["std_z"], r["bias_z"],
               r["median15_bias_z"], r["median15_std_z"]))


if __name__ == "__main__":
    sys.exit(main())
