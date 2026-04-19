#!/usr/bin/env python3
"""
map_diff — compare a saved Nav2 occupancy map (.pgm/.yaml) against the
HIL ground-truth obstacle catalogue and emit fidelity metrics.

Gate (specs/acceptance.yaml#hil_validation.map_fidelity):
  per-obstacle centroid error p95 <= 0.10 m
  per-obstacle IoU >= 0.70 (Hungarian-matched pairs)
  corridor traversability slack <= 5% (A* on GT free-space vs on .pgm)
  free-space precision >= 0.95 AND recall >= 0.90 (pixel-level)

Inputs:
  --pgm PATH                  Nav2 .pgm from map_saver_cli
  --yaml PATH                 Nav2 .yaml metadata (resolution, origin, image)
  --obstacles-json PATH       Ground-truth catalogue dumped from the latched
                              /agv/sim/ground_truth/obstacles topic as a JSON
                              file, with schema:
                                {"obstacles": [
                                    {"name": str, "kind": str,
                                     "pose": {"x","y","z","yaw"},
                                     "bbox": {"sx","sy","sz"}}, ...]}
  --corridor-start X,Y        Optional A* start (map frame)
  --corridor-end X,Y          Optional A* end

  --out DIR                   Output directory for map_diff_<name>.json
                              (default: dir of the .pgm)

Exit code:
  0 — all gate thresholds passed
  1 — at least one threshold failed
  2 — input error (e.g. missing files)

This script is intentionally stdlib + numpy only. It does NOT depend on
rclpy — snapshot the topic once with `ros2 topic echo --once ... > obs.json`
before invoking. See docs/validation/RUNBOOK_lan_hil.md §6.
"""
from __future__ import annotations

import argparse
import heapq
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    import numpy as np
except ImportError:
    print("map_diff: numpy required", file=sys.stderr)
    sys.exit(2)

try:
    import yaml
except ImportError:
    print("map_diff: python3-yaml required", file=sys.stderr)
    sys.exit(2)

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    print("map_diff: scipy required (for Hungarian matching)", file=sys.stderr)
    sys.exit(2)


CENTROID_P95_GATE_M = 0.10
IOU_MIN_GATE = 0.70
CORRIDOR_SLACK_GATE = 0.05
PRECISION_GATE = 0.95
RECALL_GATE = 0.90


@dataclass
class NavMap:
    grid: np.ndarray           # 2D array, int16, Nav2 cost values
    free: np.ndarray           # bool mask (probability < free_thresh)
    occ: np.ndarray            # bool mask (probability > occupied_thresh)
    resolution: float          # m/cell
    origin_xy: tuple[float, float]  # map origin (lower-left) in map frame
    origin_yaw: float
    name: str


def _load_nav_map(pgm_path: Path, yaml_path: Path) -> NavMap:
    with yaml_path.open() as f:
        meta = yaml.safe_load(f)
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    origin_xy = (float(origin[0]), float(origin[1]))
    origin_yaw = float(origin[2]) if len(origin) > 2 else 0.0
    occ_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))
    negate = int(meta.get("negate", 0))

    raw = _read_pgm(pgm_path)
    if negate:
        p = raw.astype(np.float32) / 255.0
    else:
        p = (255 - raw.astype(np.float32)) / 255.0
    # Flip vertically — Nav2 convention: row 0 is top of image, origin is
    # lower-left in world coords.
    p = np.flipud(p)
    occ = p > occ_thresh
    free = p < free_thresh
    grid = (p * 255).astype(np.int16)
    return NavMap(
        grid=grid,
        free=free,
        occ=occ,
        resolution=resolution,
        origin_xy=origin_xy,
        origin_yaw=origin_yaw,
        name=pgm_path.stem,
    )


def _read_pgm(path: Path) -> np.ndarray:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"unsupported PGM magic {magic!r}")
        # Skip comments.
        line = f.readline().decode().strip()
        while line.startswith("#"):
            line = f.readline().decode().strip()
        w, h = [int(x) for x in line.split()]
        maxval = int(f.readline().strip())
        if maxval > 255:
            raise ValueError("16-bit PGM not supported")
        if magic == b"P5":
            data = np.frombuffer(f.read(), dtype=np.uint8, count=w * h)
        else:
            data = np.array([int(x) for x in f.read().split()], dtype=np.uint8)
    return data.reshape((h, w))


def _world_to_cell(xy: tuple[float, float], nm: NavMap) -> tuple[int, int]:
    cx = (xy[0] - nm.origin_xy[0]) / nm.resolution
    cy = (xy[1] - nm.origin_xy[1]) / nm.resolution
    return (int(round(cx)), int(round(cy)))


def _rasterize_obstacle(
    pose: dict, bbox: dict, grid_shape: tuple[int, int], nm: NavMap
) -> np.ndarray:
    """Rasterize an OBB as a boolean mask with the same shape as nm.free."""
    x, y = float(pose["x"]), float(pose["y"])
    yaw = float(pose.get("yaw", 0.0))
    sx, sy = float(bbox["sx"]), float(bbox["sy"])
    # OBB corners in world frame.
    hx, hy = sx / 2.0, sy / 2.0
    corners = np.array(
        [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]],
        dtype=np.float64,
    )
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    wc = (rot @ corners.T).T + np.array([x, y])
    # Bounding cells.
    xs = [(p[0] - nm.origin_xy[0]) / nm.resolution for p in wc]
    ys = [(p[1] - nm.origin_xy[1]) / nm.resolution for p in wc]
    cx_min = max(0, int(math.floor(min(xs))))
    cx_max = min(grid_shape[1] - 1, int(math.ceil(max(xs))))
    cy_min = max(0, int(math.floor(min(ys))))
    cy_max = min(grid_shape[0] - 1, int(math.ceil(max(ys))))
    mask = np.zeros(grid_shape, dtype=bool)
    if cx_max < cx_min or cy_max < cy_min:
        return mask
    # Inverse-transform each cell center back to OBB frame; test inclusion.
    cy_grid, cx_grid = np.meshgrid(
        np.arange(cy_min, cy_max + 1), np.arange(cx_min, cx_max + 1), indexing="ij"
    )
    wx = nm.origin_xy[0] + (cx_grid + 0.5) * nm.resolution
    wy = nm.origin_xy[1] + (cy_grid + 0.5) * nm.resolution
    dx = wx - x
    dy = wy - y
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    inside = (np.abs(lx) <= hx) & (np.abs(ly) <= hy)
    mask[cy_min:cy_max + 1, cx_min:cx_max + 1] = inside
    return mask


def _connected_components(mask: np.ndarray) -> list[np.ndarray]:
    """Simple 4-connectivity flood fill. Returns list of component masks."""
    components: list[np.ndarray] = []
    seen = np.zeros_like(mask)
    h, w = mask.shape
    for i0 in range(h):
        for j0 in range(w):
            if not mask[i0, j0] or seen[i0, j0]:
                continue
            cm = np.zeros_like(mask)
            stack = [(i0, j0)]
            while stack:
                i, j = stack.pop()
                if i < 0 or j < 0 or i >= h or j >= w:
                    continue
                if seen[i, j] or not mask[i, j]:
                    continue
                seen[i, j] = True
                cm[i, j] = True
                stack.append((i + 1, j))
                stack.append((i - 1, j))
                stack.append((i, j + 1))
                stack.append((i, j - 1))
            if cm.sum() >= 2:  # drop salt-and-pepper singletons
                components.append(cm)
    return components


def _centroid(mask: np.ndarray, nm: NavMap) -> tuple[float, float]:
    ys, xs = np.nonzero(mask)
    cy = ys.mean()
    cx = xs.mean()
    wx = nm.origin_xy[0] + (cx + 0.5) * nm.resolution
    wy = nm.origin_xy[1] + (cy + 0.5) * nm.resolution
    return wx, wy


def _iou(a: np.ndarray, b: np.ndarray) -> float:
    inter = np.logical_and(a, b).sum()
    union = np.logical_or(a, b).sum()
    return float(inter / union) if union else 0.0


def _astar(free: np.ndarray, start: tuple[int, int], end: tuple[int, int]) -> Optional[float]:
    """8-connected A* on boolean free-space mask. Returns path length in cells, or None."""
    h, w = free.shape
    sx, sy = start
    ex, ey = end
    if not (0 <= sy < h and 0 <= sx < w and 0 <= ey < h and 0 <= ex < w):
        return None
    if not free[sy, sx] or not free[ey, ex]:
        return None
    nbrs = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    g = {(sx, sy): 0.0}
    pq: list[tuple[float, tuple[int, int]]] = [(0.0, (sx, sy))]
    while pq:
        f, cur = heapq.heappop(pq)
        if cur == (ex, ey):
            return g[cur]
        cx, cy = cur
        for dx, dy in nbrs:
            nx, ny = cx + dx, cy + dy
            if not (0 <= ny < h and 0 <= nx < w):
                continue
            if not free[ny, nx]:
                continue
            step = math.sqrt(2) if dx and dy else 1.0
            ng = g[cur] + step
            nb = (nx, ny)
            if ng < g.get(nb, float("inf")):
                g[nb] = ng
                heur = math.hypot(ex - nx, ey - ny)
                heapq.heappush(pq, (ng + heur, nb))
    return None


def _parse_xy(s: Optional[str]) -> Optional[tuple[float, float]]:
    if not s:
        return None
    a, b = s.split(",")
    return float(a), float(b)


def _load_obstacles(json_path: Path) -> list[dict]:
    """Tolerant loader for the GT obstacles snapshot.

    Accepts any of:
      (a) the raw `{"obstacles": [...]}` or `{"static_obstacles": [...]}` JSON,
      (b) a ros2 topic echo wrapper `{"data": "<json string>"}`,
      (c) the output of `ros2 topic echo --field data /agv/.../obstacles`
          which prints the raw JSON on line 1 followed by a `---` separator
          line and potentially more stanzas on repeated messages.

    Normalizes each entry to `{name, kind, pose: {x,y,z,yaw}, bbox: {sx,sy,sz}}`
    regardless of the sim-side schema variant (`pose` list vs dict,
    `yaw_deg` vs `yaw`, `bbox_m` vs `bbox`).
    """
    text = json_path.read_text().strip()

    obj = None
    try:
        decoder = json.JSONDecoder()
        obj, _ = decoder.raw_decode(text.lstrip())
        if isinstance(obj, dict) and "data" in obj and "obstacles" not in obj \
                and "static_obstacles" not in obj:
            obj = json.loads(obj["data"])
    except json.JSONDecodeError:
        for line in text.splitlines():
            line = line.strip()
            if not line or line == "---":
                continue
            try:
                obj = json.loads(line)
                break
            except json.JSONDecodeError:
                continue

    if not isinstance(obj, dict):
        raise ValueError(
            f"could not extract obstacles JSON from {json_path}. "
            "Expected /agv/sim/ground_truth/obstacles payload."
        )

    raw = obj.get("obstacles") or obj.get("static_obstacles")
    if raw is None:
        raise ValueError(
            f"payload has neither 'obstacles' nor 'static_obstacles' keys: "
            f"{sorted(obj.keys())}"
        )

    return [_normalize_obstacle(e) for e in raw]


def _normalize_obstacle(e: dict) -> dict:
    name = e.get("name", "obs")
    kind = e.get("kind") or e.get("type") or "obstacle"

    pose_in = e.get("pose")
    if isinstance(pose_in, list):
        x = float(pose_in[0]); y = float(pose_in[1])
        z = float(pose_in[2]) if len(pose_in) > 2 else 0.0
    elif isinstance(pose_in, dict):
        x = float(pose_in.get("x", 0.0))
        y = float(pose_in.get("y", 0.0))
        z = float(pose_in.get("z", 0.0))
    else:
        x = y = z = 0.0

    if "yaw" in e:
        yaw = float(e["yaw"])
    elif "yaw_deg" in e:
        yaw = math.radians(float(e["yaw_deg"]))
    elif isinstance(pose_in, dict) and "yaw" in pose_in:
        yaw = float(pose_in["yaw"])
    else:
        yaw = 0.0

    bbox_in = e.get("bbox") or e.get("bbox_m")
    if isinstance(bbox_in, list):
        sx = float(bbox_in[0]); sy = float(bbox_in[1])
        sz = float(bbox_in[2]) if len(bbox_in) > 2 else 0.0
    elif isinstance(bbox_in, dict):
        sx = float(bbox_in.get("sx", 0.0))
        sy = float(bbox_in.get("sy", 0.0))
        sz = float(bbox_in.get("sz", 0.0))
    else:
        sx = sy = sz = 0.0

    return {
        "name": name,
        "kind": kind,
        "pose": {"x": x, "y": y, "z": z, "yaw": yaw},
        "bbox": {"sx": sx, "sy": sy, "sz": sz},
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Compare Nav2 map vs HIL ground-truth obstacles.")
    ap.add_argument("--pgm", type=Path, required=True)
    ap.add_argument("--yaml", type=Path, required=True)
    ap.add_argument("--obstacles-json", type=Path, required=True,
                    help="JSON dump of /agv/sim/ground_truth/obstacles payload")
    ap.add_argument("--corridor-start", type=str, default=None, help="X,Y in map frame")
    ap.add_argument("--corridor-end", type=str, default=None, help="X,Y in map frame")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    for p in (args.pgm, args.yaml, args.obstacles_json):
        if not p.is_file():
            print(f"map_diff: missing input {p}", file=sys.stderr)
            return 2

    nm = _load_nav_map(args.pgm, args.yaml)
    obstacles = _load_obstacles(args.obstacles_json)

    gt_masks: list[np.ndarray] = []
    gt_names: list[str] = []
    for obs in obstacles:
        m = _rasterize_obstacle(obs["pose"], obs["bbox"], nm.free.shape, nm)
        if m.any():
            gt_masks.append(m)
            gt_names.append(obs.get("name", f"{obs.get('kind','?')}_{len(gt_names)}"))

    pgm_components = _connected_components(nm.occ)

    # Hungarian matching: cost matrix is centroid L2 (m). Missing pair => large cost.
    BIG = 1e6
    if not gt_masks or not pgm_components:
        print("map_diff: no GT obstacles or PGM components found — cannot measure fidelity", file=sys.stderr)
        return 2
    cost = np.full((len(gt_masks), len(pgm_components)), BIG, dtype=np.float64)
    gt_centroids = [_centroid(m, nm) for m in gt_masks]
    pgm_centroids = [_centroid(m, nm) for m in pgm_components]
    for i, gc in enumerate(gt_centroids):
        for j, pc in enumerate(pgm_centroids):
            cost[i, j] = math.hypot(gc[0] - pc[0], gc[1] - pc[1])
    row_ind, col_ind = linear_sum_assignment(cost)

    centroid_errors: list[float] = []
    ious: list[float] = []
    pair_details: list[dict] = []
    for r, c in zip(row_ind, col_ind):
        d = float(cost[r, c])
        iou = _iou(gt_masks[r], pgm_components[c])
        centroid_errors.append(d)
        ious.append(iou)
        pair_details.append({
            "gt_name": gt_names[r],
            "gt_centroid_m": list(gt_centroids[r]),
            "pgm_centroid_m": list(pgm_centroids[c]),
            "centroid_error_m": d,
            "iou": iou,
        })

    # Unmatched components / obstacles
    matched_gt = set(row_ind.tolist())
    matched_pgm = set(col_ind.tolist())
    unmatched_gt = [gt_names[i] for i in range(len(gt_masks)) if i not in matched_gt]
    unmatched_pgm_count = sum(1 for j in range(len(pgm_components)) if j not in matched_pgm)

    # Corridor A* slack (optional).
    corridor = None
    start_xy = _parse_xy(args.corridor_start)
    end_xy = _parse_xy(args.corridor_end)
    if start_xy and end_xy:
        gt_free = np.ones_like(nm.free)
        for m in gt_masks:
            gt_free[m] = False
        pgm_free = nm.free.copy()
        s_cell = _world_to_cell(start_xy, nm)
        e_cell = _world_to_cell(end_xy, nm)
        gt_path = _astar(gt_free, s_cell, e_cell)
        pgm_path = _astar(pgm_free, s_cell, e_cell)
        if gt_path and pgm_path:
            slack = (pgm_path - gt_path) / gt_path
            corridor = {
                "start_xy": list(start_xy),
                "end_xy": list(end_xy),
                "gt_path_cells": gt_path,
                "pgm_path_cells": pgm_path,
                "slack": slack,
            }
        else:
            corridor = {"error": "A* failed on one or both maps"}

    # Pixel-level free-space precision/recall.
    gt_free_mask = np.ones_like(nm.free)
    for m in gt_masks:
        gt_free_mask[m] = False
    tp = np.logical_and(nm.free, gt_free_mask).sum()
    fp = np.logical_and(nm.free, ~gt_free_mask).sum()
    fn = np.logical_and(~nm.free, gt_free_mask).sum()
    precision = float(tp / (tp + fp)) if (tp + fp) else 0.0
    recall = float(tp / (tp + fn)) if (tp + fn) else 0.0

    def p95(values: list[float]) -> float:
        if not values:
            return float("nan")
        s = sorted(values)
        idx = max(0, math.ceil(0.95 * len(s)) - 1)
        return s[idx]

    summary = {
        "pairs": len(pair_details),
        "unmatched_gt": unmatched_gt,
        "unmatched_pgm_components": unmatched_pgm_count,
        "centroid_error_p95_m": p95(centroid_errors),
        "centroid_error_max_m": max(centroid_errors) if centroid_errors else float("nan"),
        "iou_min": min(ious) if ious else float("nan"),
        "iou_median": float(np.median(ious)) if ious else float("nan"),
        "free_space_precision": precision,
        "free_space_recall": recall,
        "corridor": corridor,
    }
    gates = {
        "centroid_error_p95_m": CENTROID_P95_GATE_M,
        "iou_min": IOU_MIN_GATE,
        "corridor_slack_max": CORRIDOR_SLACK_GATE,
        "free_space_precision_min": PRECISION_GATE,
        "free_space_recall_min": RECALL_GATE,
    }
    failures: list[str] = []
    if summary["centroid_error_p95_m"] > CENTROID_P95_GATE_M:
        failures.append(f"centroid p95 {summary['centroid_error_p95_m']:.3f} > {CENTROID_P95_GATE_M}")
    if not math.isnan(summary["iou_min"]) and summary["iou_min"] < IOU_MIN_GATE:
        failures.append(f"iou_min {summary['iou_min']:.3f} < {IOU_MIN_GATE}")
    if corridor and "slack" in corridor and corridor["slack"] > CORRIDOR_SLACK_GATE:
        failures.append(f"corridor slack {corridor['slack']:.3f} > {CORRIDOR_SLACK_GATE}")
    if precision < PRECISION_GATE:
        failures.append(f"free_space_precision {precision:.3f} < {PRECISION_GATE}")
    if recall < RECALL_GATE:
        failures.append(f"free_space_recall {recall:.3f} < {RECALL_GATE}")

    result = {
        "map": nm.name,
        "resolution_m_per_cell": nm.resolution,
        "origin_xy": list(nm.origin_xy),
        "gates": gates,
        "summary": summary,
        "pairs": pair_details,
        "pass": len(failures) == 0,
        "failures": failures,
    }

    out_dir = args.out if args.out else args.pgm.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"map_diff_{nm.name}.json"
    out_path.write_text(json.dumps(result, indent=2, default=str))
    print(json.dumps(result, indent=2, default=str))
    print(f"\nREPORT: {out_path}")

    return 0 if result["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
