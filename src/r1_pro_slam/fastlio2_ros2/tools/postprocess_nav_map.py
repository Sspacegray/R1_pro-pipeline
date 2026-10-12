#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PGM(2D occupancy map) post-processing for navigation:
- Connect small gaps in obstacles (morphological close)
- Remove speckle noise (morphological open)
- Optional obstacle inflation (in meters, using map resolution)
- Preserve unknown areas by default (optionally fill enclosed unknown holes)

Designed for Nav2 map_server-compatible output (PGM + YAML).
"""

from __future__ import annotations

import argparse
import os
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class MapMeta:
    image: str
    mode: str
    resolution: float
    origin: list
    negate: int
    occupied_thresh: float
    free_thresh: float


def _meters_to_radius_px(meters: float, resolution: float) -> int:
    if meters <= 0.0:
        return 0
    return max(1, int(round(meters / resolution)))


def _kernel(radius_px: int) -> np.ndarray:
    # Elliptic kernel behaves better than a box for occupancy maps.
    k = radius_px * 2 + 1
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))


def _load_yaml(path: str) -> tuple[MapMeta, dict]:
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    meta = MapMeta(
        image=str(raw["image"]),
        mode=str(raw.get("mode", "trinary")),
        resolution=float(raw["resolution"]),
        origin=list(raw["origin"]),
        negate=int(raw.get("negate", 0)),
        occupied_thresh=float(raw.get("occupied_thresh", 0.65)),
        free_thresh=float(raw.get("free_thresh", 0.25)),
    )
    return meta, raw


def _pgm_to_occ_prob(img_u8: np.ndarray, negate: int) -> np.ndarray:
    # map_server convention:
    #   if negate==0: black(0) is occupied, white(255) is free => occ = (255 - v)/255
    #   if negate==1: white is occupied, black is free => occ = v/255
    img = img_u8.astype(np.float32) / 255.0
    if negate == 0:
        return 1.0 - img
    return img


def _compose_pgm(
    occ_mask: np.ndarray, unknown_mask: np.ndarray, negate: int
) -> np.ndarray:
    out = np.full(occ_mask.shape, 254, dtype=np.uint8)  # free
    out[unknown_mask] = 205
    if negate == 0:
        out[occ_mask] = 0
    else:
        out[occ_mask] = 254
        out[~occ_mask & ~unknown_mask] = 0
        out[unknown_mask] = 205
    return out


def _fill_enclosed_unknown(occ_mask: np.ndarray, unknown_mask: np.ndarray) -> np.ndarray:
    """
    If an unknown region is fully enclosed by obstacles, convert it to occupied.
    This avoids "holes" inside walls after closing/inflation.
    """
    if not np.any(unknown_mask):
        return occ_mask

    h, w = unknown_mask.shape
    unk = (unknown_mask.astype(np.uint8) * 255)
    # Find connected components in unknown.
    n, labels = cv2.connectedComponents((unk > 0).astype(np.uint8), connectivity=8)
    if n <= 1:
        return occ_mask

    occ_out = occ_mask.copy()
    for cid in range(1, n):
        ys, xs = np.where(labels == cid)
        if ys.size == 0:
            continue
        touches_border = (ys.min() == 0) or (ys.max() == h - 1) or (xs.min() == 0) or (xs.max() == w - 1)
        if touches_border:
            continue
        # Enclosed unknown -> occupied
        occ_out[labels == cid] = True
    return occ_out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--yaml", required=True, help="Input map yaml (nav2 map_server format)")
    ap.add_argument("--out_dir", default="", help="Output directory (default: same as yaml)")
    ap.add_argument("--out_name", default="map_pp", help="Output base name (without extension)")
    ap.add_argument("--close_m", type=float, default=0.10, help="Morph close radius in meters (connect gaps)")
    ap.add_argument("--open_m", type=float, default=0.05, help="Morph open radius in meters (remove speckles)")
    ap.add_argument("--inflate_m", type=float, default=0.15, help="Obstacle inflation radius in meters")
    ap.add_argument("--fill_unknown_holes", action="store_true", help="Convert enclosed unknown holes to occupied")
    ap.add_argument("--debug_png", action="store_true", help="Write a debug overlay PNG next to output")
    args = ap.parse_args()

    meta, raw_yaml = _load_yaml(args.yaml)
    yaml_dir = os.path.dirname(os.path.abspath(args.yaml))
    img_path = os.path.join(yaml_dir, meta.image)
    if not os.path.isfile(img_path):
        raise FileNotFoundError(f"image not found: {img_path}")

    img_u8 = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img_u8 is None:
        raise RuntimeError(f"failed to read: {img_path}")

    occ_prob = _pgm_to_occ_prob(img_u8, meta.negate)
    occ0 = occ_prob > meta.occupied_thresh
    free0 = occ_prob < meta.free_thresh
    unk0 = ~(occ0 | free0)

    close_r = _meters_to_radius_px(args.close_m, meta.resolution)
    open_r = _meters_to_radius_px(args.open_m, meta.resolution)
    infl_r = _meters_to_radius_px(args.inflate_m, meta.resolution)

    occ = (occ0.astype(np.uint8) * 255)
    if close_r > 0:
        occ = cv2.morphologyEx(occ, cv2.MORPH_CLOSE, _kernel(close_r))
    if open_r > 0:
        occ = cv2.morphologyEx(occ, cv2.MORPH_OPEN, _kernel(open_r))
    if infl_r > 0:
        occ = cv2.dilate(occ, _kernel(infl_r))

    occ_mask = occ > 0
    if args.fill_unknown_holes:
        occ_mask = _fill_enclosed_unknown(occ_mask, unk0)

    out_dir = args.out_dir or yaml_dir
    os.makedirs(out_dir, exist_ok=True)
    out_pgm = os.path.join(out_dir, f"{args.out_name}.pgm")
    out_yaml = os.path.join(out_dir, f"{args.out_name}.yaml")

    out_img = _compose_pgm(occ_mask, unk0, meta.negate)
    if not cv2.imwrite(out_pgm, out_img):
        raise RuntimeError(f"failed to write: {out_pgm}")

    raw_out = dict(raw_yaml)
    raw_out["image"] = os.path.basename(out_pgm)
    with open(out_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(raw_out, f, sort_keys=False, allow_unicode=False)

    if args.debug_png:
        # Red: new occupied pixels (after processing), Blue: original occupied.
        dbg = cv2.cvtColor(img_u8, cv2.COLOR_GRAY2BGR)
        dbg[occ0] = (255, 0, 0)
        dbg[occ_mask] = (0, 0, 255)
        dbg_path = os.path.join(out_dir, f"{args.out_name}_debug.png")
        cv2.imwrite(dbg_path, dbg)

    print(f"Wrote: {out_pgm}")
    print(f"Wrote: {out_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

