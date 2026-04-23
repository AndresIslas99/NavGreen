#!/usr/bin/env python3
"""solvePnP noise benchmark for rail_approach fine_servo.

Goal: quantify tvec/rvec noise for our exact camera + tag geometry under
corner pixel noise, across OpenCV solvePnP methods, so we can decide
whether switching from SOLVEPNP_ITERATIVE to SOLVEPNP_IPPE_SQUARE (or
SQPNP) is justified — or whether the 6.7 cm analytical depth-noise floor
is geometric and no method can beat it.

Scenario parameters match the iter-42 HIL run exactly:
  ZED 2i VGA NATIVE: 672×376, fx=fy=235.3 px, cx=336, cy=188
  AprilTag tag36h11 face 20 cm (side = 0.20 m)
  Floor tag at world z = 0.002 m, yaw = 0
  Robot at rail_approach fine_servo start: base at (5.50, 0, 0.20) in world,
    yaw = π (facing -X world); tag at (3.67, 0, 0.002).
  Camera offset base → optical: (+0.70, +0.06, +0.010) base frame.

Two scenarios:
  "grazing"  — cam world z = 0.01 (the 2D-EKF TF the brain sees). Incidence
               ~89.6°. This is what ITERATIVE breaks on.
  "realistic" — cam world z = 0.21 (physical, what the iter-40 GT-based
               shim restores). Incidence ~79°.

Both run Monte Carlo with N=1000 noisy-corner realizations per method.
Metrics reported:
  - σ (std) of tvec component {x, y, z} and range ‖tvec‖
  - bias (mean − ground truth)
  - fraction of runs where solvePnP returned a planar-flipped solution
    (detected by comparing rvec to the ground-truth rvec; flipped pose
     has rvec with wrong sign on the principal axis)
  - σ after median filter of window 5 and 15 frames (rolling)

No changes to production code — this is a standalone diagnostic.
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np


# --- Scenario geometry -----------------------------------------------------

@dataclass
class Camera:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def K(self) -> np.ndarray:
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ])


ZED2i_VGA_NATIVE = Camera(fx=235.3, fy=235.3, cx=336.0, cy=188.0,
                          width=672, height=376)

TAG_SIZE_M = 0.20   # tag36h11 side length

# Corner object points in tag local frame: BL, BR, TR, TL (CCW from +Z face).
def tag_object_points() -> np.ndarray:
    h = TAG_SIZE_M / 2.0
    return np.array([
        [-h, -h, 0.0],
        [ h, -h, 0.0],
        [ h,  h, 0.0],
        [-h,  h, 0.0],
    ], dtype=np.float64)


def quat_yaw_to_R(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]])


# Transform corners in WORLD frame given tag_world pose (x, y, z, yaw).
def tag_corners_world(tag_xy_z_yaw: Tuple[float, float, float, float]) -> np.ndarray:
    x, y, z, yaw = tag_xy_z_yaw
    R_world_tag = quat_yaw_to_R(yaw)
    obj = tag_object_points()
    # Floor tag: tag +Z axis aligned with world +Z. Local (x, y, 0) maps to
    # world (Rx, Ry, z) via rotation around Z. Translate to (x, y, z).
    world = (R_world_tag @ obj.T).T + np.array([x, y, z])
    return world  # (4, 3)


def base_pose_to_cam_world(base_xy_yaw: Tuple[float, float, float],
                           base_z: float,
                           cam_in_base_xyz: Tuple[float, float, float]
                           ) -> Tuple[np.ndarray, np.ndarray]:
    bx, by, byaw = base_xy_yaw
    cx, cy, cz = cam_in_base_xyz
    c, s = math.cos(byaw), math.sin(byaw)
    R_base_world = np.array([[c, -s, 0.0],
                             [s,  c, 0.0],
                             [0.0, 0.0, 1.0]])
    base_pos = np.array([bx, by, base_z])
    cam_offset_world = R_base_world @ np.array([cx, cy, cz])
    cam_pos_world = base_pos + cam_offset_world

    # Camera-optical: X=right, Y=down, Z=forward.
    # With robot yaw, cam forward (optical Z) = robot forward (base +X) rotated.
    # Build R_world_cam (columns are optical-axis expressed in world).
    robot_forward_world = R_base_world @ np.array([1.0, 0.0, 0.0])
    robot_left_world    = R_base_world @ np.array([0.0, 1.0, 0.0])
    robot_up_world      = np.array([0.0, 0.0, 1.0])
    cam_Z_world = robot_forward_world
    cam_X_world = -robot_left_world          # optical X = right
    cam_Y_world = -robot_up_world            # optical Y = down
    R_world_cam = np.stack([cam_X_world, cam_Y_world, cam_Z_world], axis=1)
    return cam_pos_world, R_world_cam


def project_corners(corners_world: np.ndarray,
                    cam_pos_world: np.ndarray,
                    R_world_cam: np.ndarray,
                    camera: Camera) -> Tuple[np.ndarray, np.ndarray]:
    """Return (pixels Nx2, corners_in_cam_optical Nx3)."""
    # Rotate into optical frame: P_cam = R_cam_world @ (P_world - cam_pos).
    R_cam_world = R_world_cam.T
    rel = corners_world - cam_pos_world
    cam_pts = (R_cam_world @ rel.T).T
    # Pinhole project.
    pix = np.zeros((cam_pts.shape[0], 2))
    for i, (Xc, Yc, Zc) in enumerate(cam_pts):
        pix[i, 0] = camera.fx * Xc / Zc + camera.cx
        pix[i, 1] = camera.fy * Yc / Zc + camera.cy
    return pix, cam_pts


# --- Ground-truth tvec/rvec from cam/tag world poses ----------------------

def gt_tvec_rvec(corners_world: np.ndarray,
                 cam_pos_world: np.ndarray,
                 R_world_cam: np.ndarray,
                 tag_yaw: float) -> Tuple[np.ndarray, np.ndarray]:
    """Compute solvePnP ground truth: tvec = origin of tag in cam frame,
    rvec = Rodrigues of R_cam_tag. Floor tag frame: local axes = world
    axes rotated by yaw around +Z, so R_world_tag = RotZ(yaw).
    """
    tag_origin_world = corners_world.mean(axis=0)
    R_cam_world = R_world_cam.T
    tvec = R_cam_world @ (tag_origin_world - cam_pos_world)
    R_world_tag = quat_yaw_to_R(tag_yaw)
    R_cam_tag = R_cam_world @ R_world_tag
    rvec, _ = cv2.Rodrigues(R_cam_tag)
    return tvec.astype(np.float64), rvec.flatten().astype(np.float64)


# --- Benchmark core --------------------------------------------------------

METHODS = {
    "ITERATIVE":     cv2.SOLVEPNP_ITERATIVE,
    "IPPE_SQUARE":   cv2.SOLVEPNP_IPPE_SQUARE,
    "SQPNP":         cv2.SOLVEPNP_SQPNP,
    "EPNP":          cv2.SOLVEPNP_EPNP,
    "DLS":           cv2.SOLVEPNP_DLS,
}


def run_single(method_flag: int, pix: np.ndarray, obj: np.ndarray,
               K: np.ndarray, dist: np.ndarray,
               gt_rvec: np.ndarray, gt_tvec: np.ndarray
               ) -> Tuple[np.ndarray, np.ndarray, bool]:
    """Returns (tvec, rvec, flipped). flipped = pose is the planar-flip
    alternative (detected by large rvec mismatch even though tvec has
    similar range)."""
    try:
        ok, rvec, tvec = cv2.solvePnP(
            obj.astype(np.float64), pix.astype(np.float64),
            K, dist, flags=method_flag)
        if not ok:
            return None, None, False
        rvec = rvec.flatten()
        tvec = tvec.flatten()
        # Flip detection: compare rvec direction to ground truth.
        # A planar flip is approximately an 180° rotation around the
        # tag face axis; the rvec magnitude stays similar but direction
        # reverses. Use angle between rvec and gt_rvec.
        gt_norm = np.linalg.norm(gt_rvec)
        rv_norm = np.linalg.norm(rvec)
        if gt_norm > 1e-6 and rv_norm > 1e-6:
            cos = float(np.dot(gt_rvec, rvec) / (gt_norm * rv_norm))
            flipped = cos < 0.0   # opposite direction
        else:
            flipped = False
        return tvec, rvec, flipped
    except cv2.error:
        return None, None, False


@dataclass
class Scenario:
    name: str
    base_z: float
    cam_in_base: Tuple[float, float, float]


def run_benchmark(scenario: Scenario,
                  n: int = 1000,
                  noise_px: float = 1.0,
                  seed: int = 42) -> dict:
    camera = ZED2i_VGA_NATIVE
    K = camera.K()
    dist = np.zeros(4, dtype=np.float64)
    obj = tag_object_points()

    # Robot at rail_approach start pose (5.5, 0, yaw=π), tag at (3.67, 0, 0.002, yaw=0).
    base = (5.5, 0.0, math.pi)
    tag = (3.67, 0.0, 0.002, 0.0)

    corners_world = tag_corners_world(tag)
    cam_pos, R_world_cam = base_pose_to_cam_world(
        base, scenario.base_z, scenario.cam_in_base)
    pix_gt, cam_pts = project_corners(corners_world, cam_pos, R_world_cam, camera)
    gt_tvec, gt_rvec = gt_tvec_rvec(corners_world, cam_pos, R_world_cam, tag[3])
    gt_range = float(np.linalg.norm(gt_tvec))

    # Check tag in view:
    in_frame = all(0 <= u <= camera.width and 0 <= v <= camera.height
                   for u, v in pix_gt)
    tag_px_span = float(max(pix_gt[:, 0].max() - pix_gt[:, 0].min(),
                            pix_gt[:, 1].max() - pix_gt[:, 1].min()))
    # Incidence: angle between view and tag normal.
    view = cam_pos - corners_world.mean(axis=0)
    view /= np.linalg.norm(view)
    incidence_deg = math.degrees(math.acos(abs(view[2])))
    # Note: view[2] is dot with tag normal (+Z) since world +Z is the normal.

    print(f"\n=== Scenario: {scenario.name} ===")
    print(f"  base pose    = (5.50, 0.0, z={scenario.base_z}, yaw=π)")
    print(f"  cam in base  = {scenario.cam_in_base}")
    print(f"  cam in world = ({cam_pos[0]:.3f}, {cam_pos[1]:.3f}, {cam_pos[2]:.3f})")
    print(f"  tag in world = (3.67, 0.0, 0.002, yaw=0)")
    print(f"  GT tvec      = ({gt_tvec[0]:.4f}, {gt_tvec[1]:.4f}, {gt_tvec[2]:.4f})")
    print(f"  GT range     = {gt_range:.4f} m")
    print(f"  incidence    = {incidence_deg:.2f}° off-axis (tag normal vs cam-to-tag view)")
    print(f"  tag px span  = {tag_px_span:.1f} px")
    print(f"  in frame     = {in_frame}")

    rng = np.random.default_rng(seed)
    results = {}
    for name, flag in METHODS.items():
        tvecs: List[np.ndarray] = []
        rvecs: List[np.ndarray] = []
        flips = 0
        fails = 0
        for _ in range(n):
            noise = rng.normal(0.0, noise_px, pix_gt.shape)
            pix_noisy = pix_gt + noise
            tvec, rvec, flipped = run_single(
                flag, pix_noisy, obj, K, dist, gt_rvec, gt_tvec)
            if tvec is None:
                fails += 1
                continue
            tvecs.append(tvec)
            rvecs.append(rvec)
            if flipped:
                flips += 1
        if not tvecs:
            print(f"\n  {name:14s}: ALL FAILED")
            continue
        T = np.array(tvecs)            # (k, 3)
        R = np.array(rvecs)
        mean_t = T.mean(axis=0)
        std_t = T.std(axis=0)
        bias_t = mean_t - gt_tvec
        range_err = np.linalg.norm(T, axis=1) - gt_range
        # Rolling median window N.
        def rolling_median(arr: np.ndarray, w: int) -> np.ndarray:
            if len(arr) < w:
                return arr
            out = np.zeros((len(arr) - w + 1, arr.shape[1]))
            for i in range(out.shape[0]):
                for j in range(arr.shape[1]):
                    out[i, j] = np.median(arr[i:i+w, j])
            return out
        med5 = rolling_median(T, 5)
        med15 = rolling_median(T, 15)

        print(f"\n  {name:14s}  fails={fails}/{n}  flips={flips}/{len(tvecs)}  ({100*flips/max(len(tvecs),1):.1f}%)")
        print(f"    σ tvec (m)       x={std_t[0]:.5f} y={std_t[1]:.5f} z={std_t[2]:.5f}")
        print(f"    bias tvec (m)    x={bias_t[0]:+.5f} y={bias_t[1]:+.5f} z={bias_t[2]:+.5f}")
        print(f"    σ range (m)      {range_err.std():.5f}")
        print(f"    median5  σ z (m) {med5[:,2].std():.5f}")
        print(f"    median15 σ z (m) {med15[:,2].std():.5f}")
        results[name] = {
            "std_tvec": std_t.tolist(),
            "bias_tvec": bias_t.tolist(),
            "std_range": float(range_err.std()),
            "median5_std_z": float(med5[:, 2].std()),
            "median15_std_z": float(med15[:, 2].std()),
            "flips_pct": 100 * flips / max(len(tvecs), 1),
            "fails": fails,
        }
    return results


def main() -> int:
    print("solvePnP Monte-Carlo noise benchmark — rail_approach fine_servo")
    print(f"OpenCV {cv2.__version__}")
    print(f"Corner noise: ±1 px gaussian; N=1000 per method; seed=42")

    scenarios = [
        Scenario(name="grazing (cam z=0.01, EKF 2D TF)",
                 base_z=0.0,
                 cam_in_base=(0.70, 0.06, 0.010)),
        Scenario(name="realistic (cam z=0.21, GT-based shim)",
                 base_z=0.20,
                 cam_in_base=(0.70, 0.06, 0.010)),
    ]

    for sc in scenarios:
        run_benchmark(sc, n=1000, noise_px=1.0, seed=42)

    return 0


if __name__ == "__main__":
    sys.exit(main())
