#!/usr/bin/env python3
"""Build a structural Nav2 occupancy map from a PCD without changing its XY frame.

Pipeline: height filter -> XY projection -> per-cell point count -> MIN_POINTS
-> connected components -> structural selection -> noise removal -> closing.

Cells the LiDAR never observed are written as unknown (205, trinary): a cell is
observed when any PCD point (floor and ceiling included) falls in it; gaps are
closed, enclosed holes filled and only the largest observed region is kept, so
the area outside the building stays unknown instead of free.

Every occupied component is classified as
  A. structural  (large, long)        -> <prefix>.png / <prefix>.yaml
  B. noise       (a few cells)        -> discarded
  C. uncertain   (everything else)    -> <base>_residual.png only (debug)
Live obstacles (desks, carts, people) are left to the live-LiDAR local costmap.
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from PIL import Image
import yaml


RESOLUTION = 0.05
Z_MIN = 0.15
Z_MAX = 1.80
MIN_POINTS = 2
PADDING = 0.5

MIN_NOISE_CELLS = 3               # components below this are noise (B)
MIN_STATIC_COMPONENT_CELLS = 20   # structural (A) needs at least this many cells
MIN_STATIC_EXTENT_M = 1.0         # ... and this length along its principal axis
LINK_KERNEL_SIZE = 9              # groups cells <= ~0.2 m apart (dashed walls); grouping only
CLOSING_KERNEL_SIZE = 3
CLOSING_ITERATIONS = 1

KNOWN_CLOSING_KERNEL_SIZE = 7    # closes gaps <= ~0.3 m between observed cells, 0 = no unknown

PIXEL_FREE = 254
PIXEL_OCCUPIED = 0
PIXEL_UNKNOWN = 205               # map_server trinary: (255 - 205) / 255 lies between free/occupied thresh


def parse_args():
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", type=Path, default=script_dir / "lab_map.pcd")
    parser.add_argument(
        "--output-prefix",
        type=Path,
        default=script_dir / "lab_map_structural",
        help="Structural map path without .png/.yaml; a trailing '_structural' "
        "is replaced for the debug images (e.g. lab_map_residual.png)",
    )
    parser.add_argument("--resolution", type=float, default=RESOLUTION)
    parser.add_argument("--z-min", type=float, default=Z_MIN)
    parser.add_argument("--z-max", type=float, default=Z_MAX)
    parser.add_argument("--min-points", type=int, default=MIN_POINTS)
    parser.add_argument("--padding", type=float, default=PADDING)
    parser.add_argument("--min-noise-cells", type=int, default=MIN_NOISE_CELLS)
    parser.add_argument(
        "--min-static-component-cells", type=int, default=MIN_STATIC_COMPONENT_CELLS
    )
    parser.add_argument(
        "--min-static-extent-m", type=float, default=MIN_STATIC_EXTENT_M
    )
    parser.add_argument("--link-kernel", type=int, default=LINK_KERNEL_SIZE)
    parser.add_argument("--closing-kernel", type=int, default=CLOSING_KERNEL_SIZE)
    parser.add_argument(
        "--closing-iterations", type=int, default=CLOSING_ITERATIONS
    )
    parser.add_argument(
        "--known-closing-kernel",
        type=int,
        default=KNOWN_CLOSING_KERNEL_SIZE,
        help="Closing kernel [cells] of the observed-area mask; 0 marks every "
        "non-occupied cell free (no unknown, old behaviour)",
    )
    return parser.parse_args()


def validate_args(args):
    if args.resolution <= 0.0:
        raise ValueError("--resolution must be positive")
    if args.z_min >= args.z_max:
        raise ValueError("--z-min must be less than --z-max")
    if args.min_points < 1 or args.min_noise_cells < 1:
        raise ValueError("point/noise thresholds must be at least 1")
    if args.min_static_component_cells < 1 or args.min_static_extent_m < 0.0:
        raise ValueError("structural thresholds must be positive")
    for name, value in (
        ("link kernel", args.link_kernel),
        ("closing kernel", args.closing_kernel),
    ):
        if value < 1 or value % 2 == 0:
            raise ValueError(f"{name} must be a positive odd integer")
    if args.known_closing_kernel < 0 or (
        args.known_closing_kernel and args.known_closing_kernel % 2 == 0
    ):
        raise ValueError("--known-closing-kernel must be 0 or a positive odd integer")
    if args.closing_iterations < 0:
        raise ValueError("--closing-iterations cannot be negative")


def output_paths(args):
    prefix = args.output_prefix.expanduser().resolve()
    base = prefix.name
    if base.endswith("_structural"):
        base = base[: -len("_structural")]
    # Append rather than with_suffix() so prefixes containing dots survive.
    return {
        "map_png": prefix.parent / f"{prefix.name}.png",
        "map_yaml": prefix.parent / f"{prefix.name}.yaml",
        "raw_png": prefix.parent / f"{base}_raw_occupancy.png",
        "residual_png": prefix.parent / f"{base}_residual.png",
        "classes_png": prefix.parent / f"{base}_classification.png",
    }


def principal_extent(rows, cols, resolution):
    """Length of a cell set along its principal and secondary axes [m]."""
    coords = np.column_stack((cols, rows)).astype(np.float64)
    if len(coords) < 2:
        return resolution, resolution
    centered = coords - coords.mean(axis=0)
    _, _, axes = np.linalg.svd(centered, full_matrices=False)
    projected = centered @ axes.T
    spans = projected.max(axis=0) - projected.min(axis=0) + 1.0
    major = spans[0] * resolution
    minor = spans[1] * resolution if len(spans) > 1 else resolution
    return major, minor


def classify_components(occupied, args):
    """Return structural, noise and uncertain masks plus per-class counts."""
    link = np.ones((args.link_kernel, args.link_kernel), dtype=np.uint8)
    grouped = cv2.morphologyEx(occupied.astype(np.uint8), cv2.MORPH_CLOSE, link)
    count, labels = cv2.connectedComponents(grouped, connectivity=8)
    labels = np.where(occupied, labels, 0)

    structural = np.zeros_like(occupied)
    noise = np.zeros_like(occupied)
    stats = {"structural": 0, "noise": 0, "uncertain": 0}
    rows, cols = np.nonzero(labels)
    order = np.argsort(labels[rows, cols], kind="stable")
    rows, cols = rows[order], cols[order]
    bounds = np.searchsorted(labels[rows, cols], np.arange(1, count + 1))
    for label in range(1, count):
        r = rows[bounds[label - 1]:bounds[label]]
        c = cols[bounds[label - 1]:bounds[label]]
        if len(r) == 0:
            continue
        if len(r) < args.min_noise_cells:
            noise[r, c] = True
            stats["noise"] += 1
            continue
        major, _ = principal_extent(r, c, args.resolution)
        if (
            len(r) >= args.min_static_component_cells
            and major >= args.min_static_extent_m
        ):
            structural[r, c] = True
            stats["structural"] += 1
        else:
            stats["uncertain"] += 1
    uncertain = occupied & ~structural & ~noise
    return structural, noise, uncertain, stats


def observed_mask(observed_cells, kernel_size):
    """Largest closed, hole-filled region of cells containing any PCD point."""
    if kernel_size == 0:
        return np.ones_like(observed_cells)
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    closed = cv2.morphologyEx(observed_cells.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    if count < 2:
        return np.zeros_like(observed_cells)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    region = (labels == largest).astype(np.uint8)
    # Fill enclosed holes: flood the background from the (padded) border, the rest is inside.
    padded = np.pad(region, 1)
    flood = padded.copy()
    cv2.floodFill(flood, None, (0, 0), 1)
    holes = (flood == 0)[1:-1, 1:-1]
    return region.astype(bool) | holes


def outside_free_islands(free, unknown):
    """Free regions cut off from the main free space that touch unknown: floor seen through
    windows / beyond walls. Left free they become fake rooms in room segmentation."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(free.astype(np.uint8), connectivity=4)
    if count < 3:
        return np.zeros_like(free)
    main = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    near_unknown = cv2.dilate(unknown.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    touching = np.unique(labels[near_unknown & free])
    return np.isin(labels, touching[touching != main])


def save_gray(path, occupied_mask, known_mask=None):
    image = np.full(occupied_mask.shape, PIXEL_FREE, dtype=np.uint8)
    if known_mask is not None:
        image[~known_mask] = PIXEL_UNKNOWN
    image[occupied_mask] = PIXEL_OCCUPIED
    # Cartesian +Y points up while image row indices point down.
    Image.fromarray(np.flipud(image)).save(path)


def generate_map(args):
    validate_args(args)
    input_path = args.input.expanduser().resolve()
    paths = output_paths(args)

    # The established lab_map assets are inputs/reference artifacts and must survive.
    protected = {input_path.with_suffix(".png"), input_path.with_suffix(".yaml")}
    if protected & set(paths.values()):
        raise ValueError(
            "Refusing to overwrite the original map assets; use an output prefix "
            "such as lab_map_structural"
        )

    paths["map_png"].parent.mkdir(parents=True, exist_ok=True)
    print(f"Loading: {input_path}")
    points = np.asarray(o3d.io.read_point_cloud(str(input_path)).points)
    if len(points) == 0:
        raise RuntimeError(f"PCD contains no points or could not be read: {input_path}")
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) == 0:
        raise RuntimeError("PCD contains no finite points")

    obstacles = points[(points[:, 2] >= args.z_min) & (points[:, 2] <= args.z_max)]
    if len(obstacles) == 0:
        raise RuntimeError("No points remain after height filtering")

    # Bounds come from the unfiltered PCD so the map origin matches the PCD/
    # localization XY frame; nothing below crops the grid.
    x_min = points[:, 0].min() - args.padding
    x_max = points[:, 0].max() + args.padding
    y_min = points[:, 1].min() - args.padding
    y_max = points[:, 1].max() + args.padding
    width = int(np.ceil((x_max - x_min) / args.resolution))
    height = int(np.ceil((y_max - y_min) / args.resolution))

    gx = np.floor((obstacles[:, 0] - x_min) / args.resolution).astype(np.int32)
    gy = np.floor((obstacles[:, 1] - y_min) / args.resolution).astype(np.int32)
    inside = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    count_grid = np.zeros((height, width), dtype=np.uint32)
    np.add.at(count_grid, (gy[inside], gx[inside]), 1)
    occupied = count_grid >= args.min_points

    structural, noise, uncertain, stats = classify_components(occupied, args)
    final = structural
    if args.closing_iterations:
        kernel = np.ones((args.closing_kernel, args.closing_kernel), dtype=np.uint8)
        final = cv2.morphologyEx(
            structural.astype(np.uint8),
            cv2.MORPH_CLOSE,
            kernel,
            iterations=args.closing_iterations,
        ).astype(bool)
    residual = occupied & ~final

    gx_all = np.floor((points[:, 0] - x_min) / args.resolution).astype(np.int32)
    gy_all = np.floor((points[:, 1] - y_min) / args.resolution).astype(np.int32)
    observed_cells = np.zeros((height, width), dtype=bool)
    observed_cells[gy_all, gx_all] = True
    known = observed_mask(observed_cells, args.known_closing_kernel) | final
    if args.known_closing_kernel:
        known &= ~outside_free_islands(known & ~final, ~known)

    save_gray(paths["map_png"], final, known)
    save_gray(paths["raw_png"], occupied)
    save_gray(paths["residual_png"], residual)
    classes = np.full((height, width, 3), 255, dtype=np.uint8)
    classes[~known] = (PIXEL_UNKNOWN,) * 3
    classes[uncertain] = (220, 30, 30)
    classes[noise] = (30, 90, 255)
    classes[final] = (0, 0, 0)
    Image.fromarray(np.flipud(classes)).save(paths["classes_png"])

    map_yaml = {
        "image": paths["map_png"].name,
        "mode": "trinary",
        "resolution": float(args.resolution),
        "origin": [float(x_min), float(y_min), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with paths["map_yaml"].open("w", encoding="utf-8") as stream:
        yaml.safe_dump(map_yaml, stream, default_flow_style=False, sort_keys=False)

    print("\nStructural occupancy map generated")
    print(f"  finite PCD points:       {len(points)}")
    print(f"  height-filtered points:  {len(obstacles)}  (z {args.z_min}..{args.z_max})")
    print(f"  raw occupied cells:      {int(occupied.sum())}")
    print(
        f"  components:              structural {stats['structural']}, "
        f"uncertain {stats['uncertain']}, noise {stats['noise']}"
    )
    print(f"  structural cells:        {int(structural.sum())}")
    print(f"  uncertain cells:         {int(uncertain.sum())}")
    print(f"  noise cells:             {int(noise.sum())}")
    print(f"  final occupied cells:    {int(final.sum())}")
    print(f"  free cells:              {int((known & ~final).sum())}")
    print(f"  unknown cells:           {int((~known).sum())}")
    print(f"  resolution:              {args.resolution} m/cell")
    print(f"  size:                    {width} x {height}")
    print(f"  origin:                  [{x_min}, {y_min}, 0.0]")
    for key in ("map_png", "map_yaml", "raw_png", "residual_png", "classes_png"):
        print(f"  {key + ':':24s} {paths[key]}")


def main():
    generate_map(parse_args())


if __name__ == "__main__":
    main()
