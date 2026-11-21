#!/usr/bin/env python3
"""Extend official Leo description; build rough MDRS terrain, rockfield, Start Station, Underground Tunnel, and ArUco markers."""
import hashlib
import json
import math
import re
import struct
import subprocess
import xml.etree.ElementTree as E
from pathlib import Path

import cv2
import numpy as np
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'gazebo'
MESH_DIR = OUT / 'meshes'
ARUCO_TEX_DIR = OUT / 'textures' / 'aruco'
ARUCO_CONFIG = OUT / 'aruco_markers.yaml'
ASSETS = ROOT / 'assets'

TUNNEL_Y = -34.0
ENTRY_X0 = 37.0
BORE_X0 = 47.0
BORE_X1 = 61.0
EXIT_X1 = 71.0
TUNNEL_DEPTH = 1.80
TUNNEL_INNER_HALF_W = 1.70
TUNNEL_INNER_H = 2.20

STL_DTYPE = np.dtype([
    ('normal', '<f4', 3),
    ('v0', '<f4', 3),
    ('v1', '<f4', 3),
    ('v2', '<f4', 3),
    ('attr', '<u2'),
])


def smoothstep(edge0: float, edge1: float, x):
    """Smooth Hermite interpolation from 0 to 1 between edge0 and edge1."""
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def ground_at(h_grid: np.ndarray, x: float, y: float) -> float:
    """Bilinearly sample terrain height at world (x, y) from north-up 1025x1025 grid."""
    col = float(np.clip((x + 1000.0) * 1024.0 / 2000.0, 0.0, 1024.0))
    row = float(np.clip((1000.0 - y) * 1024.0 / 2000.0, 0.0, 1024.0))
    c0 = min(int(math.floor(col)), 1023)
    r0 = min(int(math.floor(row)), 1023)
    fc = col - c0
    fr = row - r0
    h00 = float(h_grid[r0, c0])
    h01 = float(h_grid[r0, c0 + 1])
    h10 = float(h_grid[r0 + 1, c0])
    h11 = float(h_grid[r0 + 1, c0 + 1])
    return (1.0 - fr) * ((1.0 - fc) * h00 + fc * h01) + fr * ((1.0 - fc) * h10 + fc * h11)


def apply_local_patch(
    dh: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    cx: float,
    cy: float,
    reach: float,
    fn,
) -> None:
    """Evaluate a radial/local terrain modification only inside its bounding index slice."""
    c0 = max(0, int(math.floor((cx - reach + 1000.0) * 1024.0 / 2000.0)))
    c1 = min(1025, int(math.ceil((cx + reach + 1000.0) * 1024.0 / 2000.0)) + 1)
    r0 = max(0, int(math.floor((1000.0 - (cy + reach)) * 1024.0 / 2000.0)))
    r1 = min(1025, int(math.ceil((1000.0 - (cy - reach)) * 1024.0 / 2000.0)) + 1)
    if r0 >= r1 or c0 >= c1:
        return
    dx = xs[None, c0:c1] - cx
    dy = ys[r0:r1, None] - cy
    dh[r0:r1, c0:c1] += fn(dx, dy)


def build_rough_terrain(z_dem: np.ndarray, sx: float, sy: float, bx: float, by: float):
    """Add multi-octave roughness, craters, ridges, and recessed underground tunnel trench."""
    h0 = np.asarray(
        Image.fromarray(np.flipud(z_dem)).resize((1025, 1025), Image.Resampling.BILINEAR),
        dtype=np.float64,
    )
    xs = np.linspace(-1000.0, 1000.0, 1025)
    ys = np.linspace(1000.0, -1000.0, 1025)
    X, Y = np.meshgrid(xs, ys)

    spawn_mask = smoothstep(2.4, 6.5, np.hypot(X - sx, Y - sy))
    seg_dx = np.clip(X - sx, 0.0, 3.0)
    lane_mask = smoothstep(1.2, 3.0, np.hypot(X - (sx + seg_dx), Y - sy))
    base_mask = smoothstep(3.0, 6.0, np.hypot(X - bx, Y - by))

    tx_dx = np.maximum(0.0, np.maximum((ENTRY_X0 - 2.0) - X, X - (EXIT_X1 + 2.0)))
    tx_dy = np.maximum(0.0, np.abs(Y - TUNNEL_Y) - 7.5)
    tunnel_mask = smoothstep(0.0, 4.0, np.hypot(tx_dx, tx_dy))
    mask = spawn_mask * lane_mask * base_mask * tunnel_mask

    rng = np.random.default_rng(101)
    dh = np.zeros_like(h0)

    for grid_n, amp in [(65, 0.55), (129, 0.32), (257, 0.18), (513, 0.09)]:
        coarse = rng.uniform(-1.0, 1.0, (grid_n, grid_n)).astype(np.float32)
        up = np.asarray(
            Image.fromarray(coarse, mode='F').resize((1025, 1025), Image.Resampling.BILINEAR),
            dtype=np.float64,
        )
        dh += amp * up

    craters_near = 0
    while craters_near < 12:
        dist = float(rng.uniform(10.0, 75.0))
        ang = float(rng.uniform(-math.pi, math.pi))
        cx = sx + dist * math.cos(ang)
        cy = sy + dist * math.sin(ang)
        rad = float(rng.uniform(4.0, 15.0))
        if math.hypot(cx - sx, cy - sy) <= rad + 5.0 or math.hypot(cx - bx, cy - by) <= rad + 5.0:
            continue
        depth = float(rng.uniform(0.35, 1.5))
        craters_near += 1

        def crater_fn(dx, dy, r=rad, d=depth):
            u = np.hypot(dx, dy) / r
            bowl = np.where(u < 1.0, -d * np.power(np.maximum(0.0, 1.0 - u * u), 1.5), 0.0)
            rim = np.where(u < 2.2, 0.25 * d * np.exp(-((u - 1.05) / 0.25) ** 2), 0.0)
            return bowl + rim

        apply_local_patch(dh, xs, ys, cx, cy, rad * 2.25, crater_fn)

    for _ in range(45):
        dist = float(rng.uniform(75.0, 350.0))
        ang = float(rng.uniform(-math.pi, math.pi))
        cx = sx + dist * math.cos(ang)
        cy = sy + dist * math.sin(ang)
        rad = float(rng.uniform(8.0, 28.0))
        depth = float(rng.uniform(0.8, 3.0))

        def crater_fn(dx, dy, r=rad, d=depth):
            u = np.hypot(dx, dy) / r
            bowl = np.where(u < 1.0, -d * np.power(np.maximum(0.0, 1.0 - u * u), 1.5), 0.0)
            rim = np.where(u < 2.2, 0.25 * d * np.exp(-((u - 1.05) / 0.25) ** 2), 0.0)
            return bowl + rim

        apply_local_patch(dh, xs, ys, cx, cy, rad * 2.25, crater_fn)

    mounds = 0
    while mounds < 20:
        dist = float(rng.uniform(8.0, 85.0))
        ang = float(rng.uniform(-math.pi, math.pi))
        mx = sx + dist * math.cos(ang)
        my = sy + dist * math.sin(ang)
        rad = float(rng.uniform(3.5, 10.0))
        if math.hypot(mx - sx, my - sy) <= rad + 5.0 or math.hypot(mx - bx, my - by) <= rad + 5.0:
            continue
        height = float(rng.uniform(0.6, 2.4))
        rot = float(rng.uniform(0.0, math.pi))
        sy_g = rad * float(rng.uniform(0.35, 0.7))
        mounds += 1

        def mound_fn(dx, dy, sx_g=rad, sy_loc=sy_g, h_amp=height, a=rot):
            ca, sa = math.cos(a), math.sin(a)
            xr = dx * ca + dy * sa
            yr = -dx * sa + dy * ca
            return h_amp * np.exp(-0.5 * ((xr / sx_g) ** 2 + (yr / sy_loc) ** 2))

        apply_local_patch(dh, xs, ys, mx, my, rad * 3.2, mound_fn)

    h_base = h0 + mask * dh
    g_entry_raw = ground_at(h_base, ENTRY_X0, TUNNEL_Y)
    g_exit_raw = ground_at(h_base, EXIT_X1, TUNNEL_Y)
    z_floor_raw = min(g_entry_raw, g_exit_raw) - TUNNEL_DEPTH

    # Carve recessed trench below ramps & underground bore so heightmap never pokes through.
    t_in = np.clip((X - ENTRY_X0) / (BORE_X0 - ENTRY_X0), 0.0, 1.0)
    t_out = np.clip((X - BORE_X1) / (EXIT_X1 - BORE_X1), 0.0, 1.0)
    z_deck = np.where(
        X < BORE_X0,
        g_entry_raw + (z_floor_raw - g_entry_raw) * t_in,
        np.where(X <= BORE_X1, z_floor_raw, z_floor_raw + (g_exit_raw - z_floor_raw) * t_out),
    )
    z_target = z_deck - 0.60
    alpha_x = smoothstep(ENTRY_X0 - 1.5, ENTRY_X0 + 0.05, X) * smoothstep(EXIT_X1 + 1.5, EXIT_X1 - 0.05, X)
    alpha_y = smoothstep(7.2, 2.7, np.abs(Y - TUNNEL_Y))
    carve_alpha = alpha_x * alpha_y
    h = h_base * (1.0 - carve_alpha) + np.minimum(h_base, z_target) * carve_alpha

    h_min = float(h.min())
    h -= h_min
    h_max = float(h.max())
    g_entry_final = g_entry_raw - h_min
    g_exit_final = g_exit_raw - h_min
    tunnel_floor_z = z_floor_raw - h_min

    q16 = np.uint16(np.clip(h / h_max, 0.0, 1.0) * 65535.0)
    Image.fromarray(q16).save(OUT / 'heightmap.png')
    Image.fromarray(np.full((8, 8, 3), [128, 128, 255], np.uint8)).save(OUT / 'flat_normal.png')

    gy_row, gx = np.gradient(h, 2000.0 / 1024.0)
    gy = -gy_row
    slope = np.hypot(gx, gy)
    normal = np.sqrt(1.0 + gx * gx + gy * gy)
    shade = np.clip((0.8 - 0.9 * gx + 0.55 * gy) / normal, 0.30, 1.25)
    t = h / h_max
    base = (
        np.array([194.0, 150.0, 104.0])[None, None, :] * (1.0 - t[..., None])
        + np.array([148.0, 90.0, 60.0])[None, None, :] * t[..., None]
    )
    rock_col = np.array([102.0, 72.0, 54.0])[None, None, :]
    rock_blend = np.clip((slope - 0.14) / 0.35, 0.0, 0.72)[..., None]
    base = base * (1.0 - rock_blend) + rock_col * rock_blend
    bands = 1.0 + 0.055 * np.sin(h * 1.1) + 0.025 * np.sin(h * 3.1)
    tex_rng = np.random.default_rng(17)
    noise = tex_rng.normal(0.0, 2.5, (*h.shape, 1))
    rgb = np.clip(base * (0.55 + 0.55 * shade[..., None]) * bands[..., None] + noise, 0, 255).astype(np.uint8)
    Image.fromarray(rgb).save(OUT / 'terrain_texture.png')

    h_quant = q16.astype(np.float64) / 65535.0 * h_max
    return h_quant, h_max, g_entry_final, g_exit_final, tunnel_floor_z


def icosahedron_sub1():
    """Return unit-sphere (42x3 vertices, 80x3 outward-CCW face indices)."""
    phi = (1.0 + math.sqrt(5.0)) / 2.0
    raw_verts = [
        (-1, phi, 0), (1, phi, 0), (-1, -phi, 0), (1, -phi, 0),
        (0, -1, phi), (0, 1, phi), (0, -1, -phi), (0, 1, -phi),
        (phi, 0, -1), (phi, 0, 1), (-phi, 0, -1), (-phi, 0, 1),
    ]
    verts = [np.array(v, dtype=np.float64) / np.linalg.norm(v) for v in raw_verts]
    raw_faces = [
        (0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
        (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
        (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
        (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1),
    ]
    midpoint_cache = {}

    def midpoint(i0: int, i1: int) -> int:
        key = (min(i0, i1), max(i0, i1))
        if key in midpoint_cache:
            return midpoint_cache[key]
        m = (verts[i0] + verts[i1]) * 0.5
        m /= np.linalg.norm(m)
        idx = len(verts)
        verts.append(m)
        midpoint_cache[key] = idx
        return idx

    faces = []
    for a, b, c in raw_faces:
        ab = midpoint(a, b)
        bc = midpoint(b, c)
        ca = midpoint(c, a)
        faces.extend([(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)])
    return np.asarray(verts, dtype=np.float64), np.asarray(faces, dtype=np.int32)


def random_rotation(rng: np.random.Generator) -> np.ndarray:
    """Sample a uniform random 3x3 rotation matrix."""
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def make_rock_records(
    rng: np.random.Generator,
    base_verts: np.ndarray,
    base_faces: np.ndarray,
    h_grid: np.ndarray,
    x: float,
    y: float,
    rx: float,
    ry: float,
    rz: float,
    embed_frac: float,
) -> np.ndarray:
    """Generate 80 outward-oriented STL triangle records for one deformed rock."""
    v = base_verts.copy()
    dirs = rng.normal(size=(3, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    freqs = rng.uniform(2.0, 4.2, size=3)
    phases = rng.uniform(0.0, 2.0 * math.pi, size=3)
    scale = (
        1.0
        + 0.17 * np.sin(freqs[0] * (v @ dirs[0]) + phases[0])
        + 0.11 * np.cos(freqs[1] * (v @ dirs[1]) + phases[1])
        + 0.08 * np.sin(freqs[2] * (v @ dirs[2]) + phases[2])
    )
    v *= scale[:, None]

    planes = rng.normal(size=(3, 3))
    planes /= np.linalg.norm(planes, axis=1, keepdims=True)
    cuts = rng.uniform(0.50, 0.78, size=3)
    for p, c in zip(planes, cuts):
        proj = v @ p
        over = proj > c
        if np.any(over):
            v[over] -= (proj[over] - c)[:, None] * p

    v *= np.array([rx, ry, rz])[None, :]
    v = v @ random_rotation(rng).T

    rh = float(np.hypot(v[:, 0], v[:, 1]).max())
    z_center = ground_at(h_grid, x, y)
    footprint_z = [z_center]
    for k in range(8):
        ang = k * math.pi / 4.0
        footprint_z.append(ground_at(h_grid, x + 0.7 * rh * math.cos(ang), y + 0.7 * rh * math.sin(ang)))
    z_min = min(footprint_z)
    height = float(v[:, 2].max() - v[:, 2].min())
    z_c = min(
        z_center - float(v[:, 2].min()) - embed_frac * height,
        z_min - float(v[:, 2].min()) - 0.04,
    )
    center = np.array([x, y, z_c], dtype=np.float64)
    v += center[None, :]

    tris = v[base_faces].copy()
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    centroids = tris.mean(axis=1) - center[None, :]
    flip = np.sum(normals * centroids, axis=1) < 0.0
    if np.any(flip):
        v1 = tris[flip, 1].copy()
        tris[flip, 1] = tris[flip, 2]
        tris[flip, 2] = v1
        normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])

    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    valid = norms[:, 0] > 1e-12
    tris = tris[valid]
    normals = normals[valid] / norms[valid]

    rec = np.zeros(len(tris), dtype=STL_DTYPE)
    rec['normal'] = normals.astype(np.float32)
    rec['v0'] = tris[:, 0].astype(np.float32)
    rec['v1'] = tris[:, 1].astype(np.float32)
    rec['v2'] = tris[:, 2].astype(np.float32)
    return rec


def write_binary_stl(path: Path, records: list[np.ndarray], header_label: str) -> int:
    """Write combined triangle records to a little-endian binary STL file."""
    combined = np.concatenate(records) if records else np.zeros(0, dtype=STL_DTYPE)
    header = header_label.encode('ascii', 'ignore')[:80].ljust(80, b' ')
    with path.open('wb') as f:
        f.write(header)
        f.write(struct.pack('<I', len(combined)))
        combined.tofile(f)
    return len(combined)


def rock_is_clear(
    x: float,
    y: float,
    r_horiz: float,
    sx: float,
    sy: float,
    bx: float,
    by: float,
    marker_xy: list[tuple[float, float]],
) -> bool:
    """Check spawn, driving lane, start station, tunnel corridor, and ArUco marker exclusions."""
    if math.hypot(x - sx, y - sy) < 2.6 + r_horiz:
        return False
    if (sx - 0.5) <= x <= (sx + 4.5) and abs(y - sy) < 1.8 + r_horiz:
        return False
    if math.hypot(x - bx, y - by) < 4.8 + r_horiz:
        return False
    if (ENTRY_X0 - 2.5) <= x <= (EXIT_X1 + 3.0) and abs(y - TUNNEL_Y) < 8.5 + r_horiz:
        return False
    for mx, my in marker_xy:
        if math.hypot(x - mx, y - my) < 1.2 + r_horiz:
            return False
    return True


def build_rockfield(
    h_grid: np.ndarray,
    sx: float,
    sy: float,
    bx: float,
    by: float,
    marker_xy: list[tuple[float, float]],
) -> dict:
    """Generate ~420 multi-scale rocks merged into 3 material STL files."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(42)
    base_verts, base_faces = icosahedron_sub1()

    buckets = {'basalt': [], 'sandstone': [], 'ochre': []}
    counts = {'cobbles': 0, 'medium_clusters': 0, 'boulders': 0}

    def add_rock(x: float, y: float, rx: float, ry: float, rz: float, embed: float) -> bool:
        r_horiz = max(rx, ry) * 1.25
        if not rock_is_clear(x, y, r_horiz, sx, sy, bx, by, marker_xy):
            return False
        rec = make_rock_records(rng, base_verts, base_faces, h_grid, x, y, rx, ry, rz, embed)
        mat = rng.choice(['basalt', 'sandstone', 'ochre'], p=[0.35, 0.35, 0.30])
        buckets[mat].append(rec)
        return True

    attempts = 0
    while counts['cobbles'] < 160 and attempts < 1200:
        attempts += 1
        dist = float(rng.uniform(2.8, 30.0))
        ang = float(rng.uniform(-math.pi, math.pi))
        rx, ry = float(rng.uniform(0.14, 0.35)), float(rng.uniform(0.14, 0.35))
        rz = float(rng.uniform(0.09, 0.24))
        if add_rock(sx + dist * math.cos(ang), sy + dist * math.sin(ang), rx, ry, rz, 0.40):
            counts['cobbles'] += 1

    cluster_idx = 0
    while counts['medium_clusters'] < 180 and cluster_idx < 250:
        cluster_idx += 1
        dist = float(rng.uniform(4.5, 65.0))
        ang = float(rng.uniform(-math.pi, math.pi))
        cx = sx + dist * math.cos(ang)
        cy = sy + dist * math.sin(ang)
        n_in_cluster = int(rng.integers(3, 8))
        for _ in range(n_in_cluster):
            if counts['medium_clusters'] >= 180:
                break
            ox = cx + float(rng.normal(0.0, 1.3))
            oy = cy + float(rng.normal(0.0, 1.3))
            rx, ry = float(rng.uniform(0.35, 0.95)), float(rng.uniform(0.35, 0.95))
            rz = float(rng.uniform(0.25, 0.75))
            if add_rock(ox, oy, rx, ry, rz, 0.35):
                counts['medium_clusters'] += 1

    for sector in range(16):
        a0 = -math.pi + sector * (2.0 * math.pi / 16.0)
        a1 = a0 + (2.0 * math.pi / 16.0)
        for _ in range(30):
            dist = float(rng.uniform(7.0, 25.0))
            ang = float(rng.uniform(a0, a1))
            rx, ry = float(rng.uniform(0.9, 2.2)), float(rng.uniform(0.9, 2.2))
            rz = float(rng.uniform(0.7, 1.9))
            if add_rock(sx + dist * math.cos(ang), sy + dist * math.sin(ang), rx, ry, rz, 0.32):
                counts['boulders'] += 1
                break

    attempts = 0
    while counts['boulders'] < 80 and attempts < 800:
        attempts += 1
        dist = float(rng.uniform(12.0, 110.0))
        ang = float(rng.uniform(-math.pi, math.pi))
        rx, ry = float(rng.uniform(1.0, 2.8)), float(rng.uniform(1.0, 2.8))
        rz = float(rng.uniform(0.8, 2.4))
        if add_rock(sx + dist * math.cos(ang), sy + dist * math.sin(ang), rx, ry, rz, 0.32):
            counts['boulders'] += 1

    tris = {}
    for mat, recs in buckets.items():
        tris[mat] = write_binary_stl(MESH_DIR / f'rocks_{mat}.stl', recs, f'MDRS rough {mat} rocks')
    return {'counts': counts, 'total_rocks': sum(counts.values()), 'triangles': tris}


def add_primitive(
    link: E.Element,
    name: str,
    pose: str,
    geom_xml: str,
    rgba: str,
    collide: bool = True,
    friction: float | None = None,
) -> None:
    """Append visual (and optional collision) geometry to an SDF link."""
    vis = E.fromstring(
        f'<visual name="{name}_visual"><pose>{pose}</pose><geometry>{geom_xml}</geometry>'
        f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
        f'<specular>0.12 0.12 0.12 1</specular></material></visual>'
    )
    link.append(vis)
    if collide:
        surf = (
            f'<surface><friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction></surface>'
            if friction is not None
            else ''
        )
        col = E.fromstring(
            f'<collision name="{name}_collision"><pose>{pose}</pose>'
            f'<geometry>{geom_xml}</geometry>{surf}</collision>'
        )
        link.append(col)


def add_ramp_slab(
    link: E.Element,
    name: str,
    x0: float,
    z0: float,
    x1: float,
    z1: float,
    y_center: float,
    width: float,
    thickness: float,
    rgba: str,
    friction: float | None = 1.0,
) -> None:
    """Place a box whose top surface passes exactly through (x0, z0) and (x1, z1)."""
    dx, dz = x1 - x0, z1 - z0
    length = math.hypot(dx, dz)
    pitch = -math.atan2(dz, dx)
    mx, mz = 0.5 * (x0 + x1), 0.5 * (z0 + z1)
    cx = mx - 0.5 * thickness * math.sin(pitch)
    cz = mz - 0.5 * thickness * math.cos(pitch)
    add_primitive(
        link,
        name,
        f'{cx:.6f} {y_center:.6f} {cz:.6f} 0 {pitch:.6f} 0',
        f'<box><size>{length:.6f} {width:.6f} {thickness:.6f}</size></box>',
        rgba,
        collide=True,
        friction=friction,
    )


def build_overburden_stl(h_grid: np.ndarray, tunnel_floor_z: float) -> Path:
    """Build watertight earthen overburden mound STL burying the underground tunnel."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    x0, x1 = BORE_X0 - 0.18, BORE_X1 + 0.18
    half_w = 8.2
    xs = np.linspace(x0, x1, 23)
    ys = np.linspace(TUNNEL_Y - half_w, TUNNEL_Y + half_w, 33)
    nx, ny = len(xs), len(ys)

    top_pts = np.zeros((nx, ny, 3), dtype=np.float64)
    bot_pts = np.zeros((nx, ny, 3), dtype=np.float64)
    z_crest = tunnel_floor_z + 3.85
    z_arch_bot = tunnel_floor_z + 2.30

    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            dy = abs(y - TUNNEL_Y)
            w_y = float(smoothstep(half_w, 2.2, dy))
            g = ground_at(h_grid, x, y)
            z_edge = g - 0.40
            bump = 0.12 * math.sin(0.9 * x + 0.5 * y) + 0.08 * math.cos(1.3 * x - 0.8 * y)
            z_top = z_edge * (1.0 - w_y) + (z_crest + bump) * w_y
            b_blend = float(smoothstep(2.08, 2.40, dy))
            z_bot = z_arch_bot * (1.0 - b_blend) + (g - 0.65) * b_blend
            if z_top < z_bot + 0.12:
                z_top = z_bot + 0.12
            top_pts[i, j] = [x, y, z_top]
            bot_pts[i, j] = [x, y, z_bot]

    tris = []

    def add_quad(a, b, c, d, outward_hint):
        for p0, p1, p2 in ((a, b, c), (a, c, d)):
            n = np.cross(p1 - p0, p2 - p0)
            if np.linalg.norm(n) <= 1e-12:
                continue
            if np.dot(n, outward_hint) < 0.0:
                tris.append((p0, p2, p1))
            else:
                tris.append((p0, p1, p2))

    for i in range(nx - 1):
        for j in range(ny - 1):
            add_quad(top_pts[i, j], top_pts[i + 1, j], top_pts[i + 1, j + 1], top_pts[i, j + 1], np.array([0, 0, 1.0]))
            add_quad(bot_pts[i, j], bot_pts[i, j + 1], bot_pts[i + 1, j + 1], bot_pts[i + 1, j], np.array([0, 0, -1.0]))

    for j in range(ny - 1):
        add_quad(bot_pts[0, j], top_pts[0, j], top_pts[0, j + 1], bot_pts[0, j + 1], np.array([-1.0, 0, 0]))
        add_quad(bot_pts[-1, j], bot_pts[-1, j + 1], top_pts[-1, j + 1], top_pts[-1, j], np.array([1.0, 0, 0]))

    for i in range(nx - 1):
        add_quad(bot_pts[i, 0], bot_pts[i + 1, 0], top_pts[i + 1, 0], top_pts[i, 0], np.array([0, -1.0, 0]))
        add_quad(bot_pts[i, -1], top_pts[i, -1], top_pts[i + 1, -1], bot_pts[i + 1, -1], np.array([0, 1.0, 0]))

    tri_arr = np.asarray(tris, dtype=np.float64)
    normals = np.cross(tri_arr[:, 1] - tri_arr[:, 0], tri_arr[:, 2] - tri_arr[:, 0])
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= np.maximum(norms, 1e-12)

    rec = np.zeros(len(tri_arr), dtype=STL_DTYPE)
    rec['normal'] = normals.astype(np.float32)
    rec['v0'] = tri_arr[:, 0].astype(np.float32)
    rec['v1'] = tri_arr[:, 1].astype(np.float32)
    rec['v2'] = tri_arr[:, 2].astype(np.float32)

    stl_path = MESH_DIR / 'tunnel_overburden.stl'
    write_binary_stl(stl_path, [rec], 'MDRS underground tunnel overburden')
    return stl_path


def build_underground_tunnel(
    h_grid: np.ndarray,
    g_entry_final: float,
    g_exit_final: float,
    tunnel_floor_z: float,
) -> E.Element:
    """Build drivable underground tunnel, entry/exit ramps, portal piers, and overburden mound."""
    model = E.Element('model', {'name': 'underground_tunnel'})
    E.SubElement(model, 'static').text = 'true'
    link = E.SubElement(model, 'link', {'name': 'structure'})

    deck_rgba = '0.26 0.26 0.28 1'
    wall_rgba = '0.36 0.35 0.33 1'
    portal_rgba = '0.42 0.40 0.37 1'
    ret_rgba = '0.32 0.31 0.29 1'

    g_in_outer = ground_at(h_grid, ENTRY_X0 - 2.0, TUNNEL_Y)
    g_out_outer = ground_at(h_grid, EXIT_X1 + 2.0, TUNNEL_Y)

    # Smooth continuous driving decks (4.24 m wide)
    add_ramp_slab(link, 'entry_apron', ENTRY_X0 - 2.0, g_in_outer, ENTRY_X0 + 0.05, g_entry_final, TUNNEL_Y, 4.24, 0.25, deck_rgba)
    add_ramp_slab(link, 'entry_ramp', ENTRY_X0, g_entry_final, BORE_X0 + 0.05, tunnel_floor_z, TUNNEL_Y, 4.24, 0.28, deck_rgba)
    add_ramp_slab(link, 'tunnel_floor', BORE_X0 - 0.05, tunnel_floor_z, BORE_X1 + 0.05, tunnel_floor_z, TUNNEL_Y, 4.24, 0.28, deck_rgba)
    add_ramp_slab(link, 'exit_ramp', BORE_X1 - 0.05, tunnel_floor_z, EXIT_X1, g_exit_final, TUNNEL_Y, 4.24, 0.28, deck_rgba)
    add_ramp_slab(link, 'exit_apron', EXIT_X1 - 0.05, g_exit_final, EXIT_X1 + 2.0, g_out_outer, TUNNEL_Y, 4.24, 0.25, deck_rgba)

    # Bore side walls & ceiling (14.0 m long, 3.40 m clear width, 2.20 m headroom)
    bore_len = BORE_X1 - BORE_X0
    bore_cx = 0.5 * (BORE_X0 + BORE_X1)
    for side, y_off in [('left', 1.88), ('right', -1.88)]:
        add_primitive(
            link,
            f'bore_wall_{side}',
            f'{bore_cx} {TUNNEL_Y + y_off} {tunnel_floor_z + 0.975} 0 0 0',
            f'<box><size>{bore_len} 0.36 2.55</size></box>',
            wall_rgba,
            collide=True,
        )
    add_primitive(
        link,
        'bore_roof',
        f'{bore_cx} {TUNNEL_Y} {tunnel_floor_z + 2.38} 0 0 0',
        f'<box><size>{bore_len} 4.12 0.36</size></box>',
        wall_rgba,
        collide=True,
    )

    # Front and rear portal headwalls (piers + lintel)
    for tag, px in [('front', BORE_X0), ('rear', BORE_X1)]:
        for side, y_off in [('left', 2.03), ('right', -2.03)]:
            add_primitive(
                link,
                f'portal_{tag}_pier_{side}',
                f'{px} {TUNNEL_Y + y_off} {tunnel_floor_z + 1.10} 0 0 0',
                '<box><size>0.40 0.66 2.80</size></box>',
                portal_rgba,
                collide=True,
            )
        add_primitive(
            link,
            f'portal_{tag}_lintel',
            f'{px} {TUNNEL_Y} {tunnel_floor_z + 2.45} 0 0 0',
            '<box><size>0.42 4.72 0.50</size></box>',
            portal_rgba,
            collide=True,
        )

    # Sloped retaining walls ("sidebars") along both ramps (inner face at ty +- 2.04 m, flush with piers)
    for side, y_off in [('left', 2.20), ('right', -2.20)]:
        add_ramp_slab(
            link,
            f'entry_retaining_{side}',
            ENTRY_X0, g_entry_final + 0.80,
            BORE_X0 - 0.20, tunnel_floor_z + 2.50,
            TUNNEL_Y + y_off,
            0.32, 2.80, ret_rgba, friction=0.6,
        )
        add_ramp_slab(
            link,
            f'exit_retaining_{side}',
            BORE_X1 + 0.20, tunnel_floor_z + 2.50,
            EXIT_X1, g_exit_final + 0.80,
            TUNNEL_Y + y_off,
            0.32, 2.80, ret_rgba, friction=0.6,
        )
        add_primitive(
            link,
            f'entry_sidebar_pillar_{side}',
            f'{ENTRY_X0 - 0.14} {TUNNEL_Y + y_off} {g_entry_final + 0.15} 0 0 0',
            '<box><size>0.36 0.42 1.40</size></box>',
            portal_rgba,
            collide=True,
        )
        add_primitive(
            link,
            f'exit_sidebar_pillar_{side}',
            f'{EXIT_X1 + 0.14} {TUNNEL_Y + y_off} {g_exit_final + 0.15} 0 0 0',
            '<box><size>0.36 0.42 1.40</size></box>',
            portal_rgba,
            collide=True,
        )

    # Earthen overburden mound burying the tunnel
    overburden_stl = build_overburden_stl(h_grid, tunnel_floor_z)
    link.append(E.fromstring(
        f'<collision name="overburden_collision"><geometry><mesh><uri>{overburden_stl}</uri></mesh></geometry>'
        '<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>'
    ))
    link.append(E.fromstring(
        f'<visual name="overburden_visual"><geometry><mesh><uri>{overburden_stl}</uri></mesh></geometry>'
        '<material><ambient>0.50 0.34 0.22 1</ambient><diffuse>0.50 0.34 0.22 1</diffuse>'
        '<specular>0.04 0.04 0.04 1</specular></material></visual>'
    ))
    return model


def write_aruco_panel_objs() -> tuple[Path, Path]:
    """Write unit 1x1 m front and double-sided UV-mapped OBJ meshes for ArUco faces."""
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    front_obj = MESH_DIR / 'aruco_face_front.obj'
    front_obj.write_text(
        '# Unit 1x1m front ArUco face at x=+0.0085\n'
        'v 0.0085 -0.5 -0.5\n'
        'v 0.0085  0.5 -0.5\n'
        'v 0.0085  0.5  0.5\n'
        'v 0.0085 -0.5  0.5\n'
        'vt 0.0 0.0\n'
        'vt 1.0 0.0\n'
        'vt 1.0 1.0\n'
        'vt 0.0 1.0\n'
        'vn 1.0 0.0 0.0\n'
        'f 1/1/1 2/2/1 3/3/1\n'
        'f 1/1/1 3/3/1 4/4/1\n'
    )
    double_obj = MESH_DIR / 'aruco_face_double.obj'
    double_obj.write_text(
        '# Unit 1x1m double-sided ArUco faces at x=+-0.0085\n'
        'v 0.0085 -0.5 -0.5\n'
        'v 0.0085  0.5 -0.5\n'
        'v 0.0085  0.5  0.5\n'
        'v 0.0085 -0.5  0.5\n'
        'v -0.0085  0.5 -0.5\n'
        'v -0.0085 -0.5 -0.5\n'
        'v -0.0085 -0.5  0.5\n'
        'v -0.0085  0.5  0.5\n'
        'vt 0.0 0.0\n'
        'vt 1.0 0.0\n'
        'vt 1.0 1.0\n'
        'vt 0.0 1.0\n'
        'vn 1.0 0.0 0.0\n'
        'vn -1.0 0.0 0.0\n'
        'f 1/1/1 2/2/1 3/3/1\n'
        'f 1/1/1 3/3/1 4/4/1\n'
        'f 5/1/2 6/2/2 7/3/2\n'
        'f 5/1/2 7/3/2 8/4/2\n'
    )
    return front_obj, double_obj


def build_aruco_markers(
    h_grid: np.ndarray,
    g_entry_final: float,
    g_exit_final: float,
    tunnel_floor_z: float,
) -> tuple[E.Element, list[dict]]:
    """Generate ArUco marker PNGs and static SDF model from gazebo/aruco_markers.yaml."""
    ARUCO_TEX_DIR.mkdir(parents=True, exist_ok=True)
    front_obj, double_obj = write_aruco_panel_objs()

    cfg = yaml.safe_load(ARUCO_CONFIG.read_text())
    defaults = cfg.get('defaults', {})
    default_dict = str(defaults.get('dictionary', 'DICT_4X4_50'))
    default_size = defaults.get('size_cm', [20.0, 20.0])
    border_bits = int(defaults.get('border_bits', 1))
    margin_ratio = float(defaults.get('white_margin_ratio', 0.15))
    panel_thick = float(defaults.get('panel_thickness_m', 0.015))
    default_z_ground = float(defaults.get('z_above_ground', 0.45))

    model = E.Element('model', {'name': 'aruco_markers'})
    E.SubElement(model, 'static').text = 'true'
    link = E.SubElement(model, 'link', {'name': 'markers'})

    resolved = []
    for entry in cfg.get('markers', []):
        name = str(entry['name'])
        marker_id = int(entry['id'])
        dict_name = str(entry.get('dictionary', default_dict))
        dict_id = getattr(cv2.aruco, dict_name)
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        raw_size = entry.get('size_cm', default_size)
        if isinstance(raw_size, (int, float)):
            sw_cm = sh_cm = float(raw_size)
        else:
            sw_cm, sh_cm = float(raw_size[0]), float(raw_size[1])

        marker_w_m = sw_cm / 100.0
        marker_h_m = sh_cm / 100.0
        board_w_m = marker_w_m * (1.0 + 2.0 * margin_ratio)
        board_h_m = marker_h_m * (1.0 + 2.0 * margin_ratio)

        # Render high-contrast marker PNG with white quiet-zone border
        px = 400
        img_gray = cv2.aruco.generateImageMarker(aruco_dict, marker_id, px, borderBits=border_bits)
        pad = int(round(px * margin_ratio))
        padded = np.pad(img_gray, ((pad, pad), (pad, pad)), mode='constant', constant_values=255)
        png_path = ARUCO_TEX_DIR / f'{name}.png'
        Image.fromarray(cv2.cvtColor(padded, cv2.COLOR_GRAY2RGB)).save(png_path)

        pos = entry['position']
        wx, wy = float(pos['x']), float(pos['y'])
        if pos.get('z_mode') == 'tunnel_floor':
            wz = tunnel_floor_z + float(pos.get('z_offset', 0.65))
        elif pos.get('z_mode') == 'tunnel_entry':
            wz = g_entry_final + float(pos.get('z_offset', 0.48))
        elif pos.get('z_mode') == 'tunnel_exit':
            wz = g_exit_final + float(pos.get('z_offset', 0.48))
        elif 'z' in pos:
            wz = float(pos['z'])
        else:
            wz = ground_at(h_grid, wx, wy) + float(pos.get('z_above_ground', default_z_ground))

        yaw_deg = float(entry.get('yaw_deg', 0.0))
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(float(entry.get('pitch_deg', 0.0)))
        roll_rad = math.radians(float(entry.get('roll_deg', 0.0)))
        pose = f'{wx:.6f} {wy:.6f} {wz:.6f} {roll_rad:.6f} {pitch_rad:.6f} {yaw_rad:.6f}'

        double_sided = bool(entry.get('double_sided', defaults.get('double_sided', False)))
        with_post = bool(entry.get('with_post', defaults.get('with_post', True)))

        # White backing plate (collision + visual)
        add_primitive(
            link,
            f'{name}_board',
            pose,
            f'<box><size>{panel_thick:.6f} {board_w_m:.6f} {board_h_m:.6f}</size></box>',
            '0.94 0.94 0.94 1',
            collide=True,
        )

        # Textured ArUco face mesh
        face_mesh = double_obj if double_sided else front_obj
        link.append(E.fromstring(
            f'<visual name="{name}_face_visual"><pose>{pose}</pose>'
            f'<geometry><mesh><uri>{face_mesh}</uri>'
            f'<scale>1.0 {board_w_m:.6f} {board_h_m:.6f}</scale></mesh></geometry>'
            '<material><ambient>0.92 0.92 0.92 1</ambient><diffuse>0.92 0.92 0.92 1</diffuse>'
            '<specular>0.02 0.02 0.02 1</specular>'
            f'<pbr><metal><albedo_map>{png_path}</albedo_map><roughness>0.95</roughness>'
            '<metalness>0.0</metalness></metal></pbr></material></visual>'
        ))

        if with_post:
            z_top = wz - board_h_m * 0.5 + 0.01
            z_bot = min(ground_at(h_grid, wx, wy), wz - board_h_m * 0.5) - 0.30
            post_len = max(0.20, z_top - z_bot)
            add_primitive(
                link,
                f'{name}_post',
                f'{wx:.6f} {wy:.6f} {(z_top + z_bot) * 0.5:.6f} 0 0 0',
                f'<cylinder><radius>0.022</radius><length>{post_len:.6f}</length></cylinder>',
                '0.55 0.58 0.62 1',
                collide=True,
            )

        resolved.append({
            'name': name,
            'id': marker_id,
            'dictionary': dict_name,
            'size_cm': [sw_cm, sh_cm],
            'board_size_m': [board_w_m, board_h_m],
            'world_xyz': [wx, wy, wz],
            'yaw_deg': yaw_deg,
            'with_post': with_post,
            'double_sided': double_sided,
            'role': str(entry.get('role', '')),
            'texture': str(png_path),
        })

    return model, resolved


def build_start_station(h_grid: np.ndarray, sx: float, sy: float, bx: float, by: float) -> E.Element:
    """Build flush start pad, 4 corner marker posts, and MDRS start outpost model."""
    model = E.Element('model', {'name': 'start_station'})
    E.SubElement(model, 'static').text = 'true'
    link = E.SubElement(model, 'link', {'name': 'base'})

    gs = ground_at(h_grid, sx, sy)
    add_primitive(
        link, 'start_pad_ring', f'{sx} {sy} {gs + 0.004} 0 0 0',
        '<cylinder><radius>1.40</radius><length>0.008</length></cylinder>',
        '0.92 0.36 0.06 1', collide=False,
    )
    add_primitive(
        link, 'start_pad_deck', f'{sx} {sy} {gs + 0.006} 0 0 0',
        '<cylinder><radius>1.32</radius><length>0.012</length></cylinder>',
        '0.18 0.20 0.23 1', collide=False,
    )
    add_primitive(
        link, 'start_chevron_l', f'{sx + 0.85} {sy + 0.08} {gs + 0.014} 0 0 -0.55',
        '<box><size>0.26 0.055 0.005</size></box>',
        '0.95 0.40 0.07 1', collide=False,
    )
    add_primitive(
        link, 'start_chevron_r', f'{sx + 0.85} {sy - 0.08} {gs + 0.014} 0 0 0.55',
        '<box><size>0.26 0.055 0.005</size></box>',
        '0.95 0.40 0.07 1', collide=False,
    )

    for idx, (dx, dy) in enumerate([(-2.2, -2.2), (-2.2, 2.2), (2.2, -2.2), (2.2, 2.2)]):
        px, py = sx + dx, sy + dy
        gp = ground_at(h_grid, px, py)
        add_primitive(
            link, f'marker_post_{idx}', f'{px} {py} {gp + 0.20} 0 0 0',
            '<cylinder><radius>0.045</radius><length>0.70</length></cylinder>',
            '0.86 0.86 0.86 1', collide=True,
        )
        add_primitive(
            link, f'marker_cap_{idx}', f'{px} {py} {gp + 0.475} 0 0 0',
            '<cylinder><radius>0.048</radius><length>0.15</length></cylinder>',
            '0.96 0.36 0.05 1', collide=False,
        )

    gb = ground_at(h_grid, bx, by)
    gb_min = min(ground_at(h_grid, bx + ox, by + oy) for ox in (-1.6, 0.0, 1.6) for oy in (-1.6, 0.0, 1.6))
    skirt_bot = gb_min - 0.50
    skirt_top = gb + 0.22
    add_primitive(
        link, 'hab_skirt', f'{bx} {by} {(skirt_bot + skirt_top) * 0.5} 0 0 0',
        f'<cylinder><radius>1.68</radius><length>{skirt_top - skirt_bot}</length></cylinder>',
        '0.18 0.20 0.22 1', collide=True,
    )
    add_primitive(
        link, 'hab_body', f'{bx} {by} {gb + 1.60} 0 0 0',
        '<cylinder><radius>1.60</radius><length>3.20</length></cylinder>',
        '0.87 0.85 0.81 1', collide=True,
    )
    add_primitive(
        link, 'hab_band', f'{bx} {by} {gb + 3.16} 0 0 0',
        '<cylinder><radius>1.65</radius><length>0.14</length></cylinder>',
        '0.88 0.36 0.08 1', collide=False,
    )
    add_primitive(
        link, 'hab_dome', f'{bx} {by} {gb + 2.95} 0 0 0',
        '<sphere><radius>1.54</radius></sphere>',
        '0.90 0.89 0.86 1', collide=True,
    )

    dist_sb = math.hypot(sx - bx, sy - by)
    ux, uy = (sx - bx) / dist_sb, (sy - by) / dist_sb
    air_yaw = math.atan2(uy, ux)
    ax, ay = bx + 1.55 * ux, by + 1.55 * uy
    add_primitive(
        link, 'hab_airlock', f'{ax} {ay} {gb + 0.90} 0 0 {air_yaw}',
        '<box><size>1.10 1.10 2.40</size></box>',
        '0.74 0.74 0.72 1', collide=True,
    )
    add_primitive(
        link, 'hab_door', f'{ax + 0.54 * ux} {ay + 0.54 * uy} {gb + 0.85} 0 0 {air_yaw}',
        '<box><size>0.06 0.74 1.72</size></box>',
        '0.22 0.25 0.29 1', collide=False,
    )

    spx, spy = bx + 2.7, by - 0.6
    gsun = ground_at(h_grid, spx, spy)
    panel_yaw = 0.30
    add_primitive(
        link, 'solar_panel', f'{spx} {spy} {gsun + 1.05} 0 0.42 {panel_yaw}',
        '<box><size>1.30 2.60 0.06</size></box>',
        '0.08 0.14 0.32 1', collide=True,
    )
    for idx, off in enumerate([-0.75, 0.75]):
        post_x = spx - off * math.sin(panel_yaw)
        post_y = spy + off * math.cos(panel_yaw)
        gp_post = ground_at(h_grid, post_x, post_y)
        top_z = gsun + 1.02
        bot_z = min(gp_post, gsun) - 0.40
        add_primitive(
            link, f'solar_post_{idx}', f'{post_x} {post_y} {(top_z + bot_z) * 0.5} 0 0 0',
            f'<cylinder><radius>0.055</radius><length>{top_z - bot_z}</length></cylinder>',
            '0.55 0.58 0.62 1', collide=True,
        )

    tx, ty = bx - 2.3, by - 1.2
    gt = ground_at(h_grid, tx, ty)
    add_primitive(
        link, 'comms_base', f'{tx} {ty} {gt + 0.02} 0 0 0',
        '<box><size>0.46 0.46 0.30</size></box>',
        '0.22 0.24 0.26 1', collide=True,
    )
    add_primitive(
        link, 'comms_mast', f'{tx} {ty} {gt + 1.75} 0 0 0',
        '<cylinder><radius>0.06</radius><length>4.50</length></cylinder>',
        '0.68 0.71 0.75 1', collide=True,
    )
    add_primitive(
        link, 'comms_crossbar', f'{tx} {ty} {gt + 3.45} 0 0 0.5',
        '<box><size>0.06 0.90 0.06</size></box>',
        '0.68 0.71 0.75 1', collide=False,
    )
    add_primitive(
        link, 'comms_dish', f'{tx + 0.22} {ty + 0.12} {gt + 3.55} 0 0.95 0.45',
        '<cylinder><radius>0.34</radius><length>0.07</length></cylinder>',
        '0.92 0.92 0.90 1', collide=False,
    )
    add_primitive(
        link, 'comms_beacon', f'{tx} {ty} {gt + 4.08} 0 0 0',
        '<sphere><radius>0.14</radius></sphere>',
        '0.98 0.36 0.05 1', collide=False,
    )

    c1x, c1y = bx - 0.8, by - 2.4
    gc1 = ground_at(h_grid, c1x, c1y)
    add_primitive(
        link, 'crate_olive', f'{c1x} {c1y} {gc1 + 0.19} 0 0 0.25',
        '<box><size>0.75 0.50 0.45</size></box>',
        '0.28 0.34 0.20 1', collide=True,
    )
    c2x, c2y = bx + 0.1, by - 2.6
    gc2 = ground_at(h_grid, c2x, c2y)
    add_primitive(
        link, 'crate_orange', f'{c2x} {c2y} {gc2 + 0.24} 0 0 -0.40',
        '<box><size>0.55 0.55 0.55</size></box>',
        '0.88 0.38 0.08 1', collide=True,
    )
    return model


def build_rockfield_model() -> E.Element:
    """Create a single static SDF model referencing the 3 merged rockfield STL files."""
    materials = {
        'basalt': '0.22 0.20 0.19 1',
        'sandstone': '0.48 0.27 0.17 1',
        'ochre': '0.52 0.39 0.26 1',
    }
    model = E.Element('model', {'name': 'rough_terrain_rocks'})
    E.SubElement(model, 'static').text = 'true'
    link = E.SubElement(model, 'link', {'name': 'rocks'})
    for mat, rgba in materials.items():
        uri = str(MESH_DIR / f'rocks_{mat}.stl')
        link.append(E.fromstring(
            f'<collision name="{mat}_collision"><geometry><mesh><uri>{uri}</uri></mesh></geometry>'
            f'<surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>'
        ))
        link.append(E.fromstring(
            f'<visual name="{mat}_visual"><geometry><mesh><uri>{uri}</uri></mesh></geometry>'
            f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>'
            f'<specular>0.06 0.06 0.06 1</specular></material></visual>'
        ))
    return model


def build_leo_model(sx: float, sy: float, ground: float):
    """Build official Leo Rover URDF with 3D LiDAR + RealSense D435 payload and convert to SDF."""
    leo = Path(get_package_share_directory('leo_description'))
    xml = xacro.process_file(str(leo / 'urdf/leo_sim.urdf.xacro')).toxml()
    robot = E.fromstring(xml)

    for joint in robot.findall('joint'):
        for mimic in joint.findall('mimic'):
            joint.remove(mimic)

    for g in list(robot.findall('gazebo')):
        if g.get('reference') == 'camera_frame':
            robot.remove(g)
    for element in list(robot):
        if (
            (element.tag == 'link' and element.get('name') in {'camera_frame', 'camera_optical_frame'})
            or (element.tag == 'joint' and element.get('name') in {'camera_joint', 'camera_optical_joint'})
        ):
            robot.remove(element)

    robot.append(E.fromstring(
        '<link name="lidar_link"><inertial><mass value=".35"/>'
        '<inertia ixx=".0005" iyy=".0005" izz=".0004" ixy="0" ixz="0" iyz="0"/></inertial>'
        '<visual name="lidar_body"><geometry><cylinder radius=".045" length=".07"/></geometry>'
        '<material name="lidar_dark"><color rgba=".08 .10 .11 1"/></material></visual>'
        '<visual name="lidar_band"><geometry><cylinder radius=".046" length=".025"/></geometry>'
        '<material name="lidar_green"><color rgba=".2 .45 .28 1"/></material></visual>'
        '<visual name="lidar_mount"><origin xyz="0 0 -.12"/><geometry>'
        '<cylinder radius=".009" length=".18"/></geometry>'
        '<material name="mount_gray"><color rgba=".4 .4 .4 1"/></material></visual></link>'
    ))
    robot.append(E.fromstring(
        '<joint name="lidar_mount_joint" type="fixed"><parent link="base_link"/>'
        '<child link="lidar_link"/><origin xyz="0 0 .25"/></joint>'
    ))
    robot.append(E.fromstring(
        '<gazebo reference="lidar_link"><sensor name="lidar_3d" type="gpu_lidar">'
        '<always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>'
        '<topic>/lidar</topic><gz_frame_id>lidar_link</gz_frame_id>'
        '<lidar><scan><horizontal><samples>1024</samples><resolution>1</resolution>'
        '<min_angle>-3.141592653589793</min_angle><max_angle>3.141592653589793</max_angle>'
        '</horizontal><vertical><samples>32</samples><resolution>1</resolution>'
        '<min_angle>-.5235987755982988</min_angle><max_angle>.2617993877991494</max_angle>'
        '</vertical></scan><range><min>.2</min><max>80</max><resolution>.01</resolution></range>'
        '</lidar></sensor></gazebo>'
    ))

    robot.append(E.fromstring(
        '<link name="camera_bottom_screw_frame"><visual name="d435_bracket">'
        '<origin xyz="0 0 -0.01325"/><geometry><box size="0.024 0.032 0.0265"/></geometry>'
        '<material name="d435_mount"><color rgba="0.18 0.19 0.21 1"/></material></visual></link>'
    ))
    robot.append(E.fromstring(
        '<joint name="camera_bottom_screw_joint" type="fixed"><parent link="base_link"/>'
        '<child link="camera_bottom_screw_frame"/><origin xyz="0.12 0.0 0.0325" rpy="0 0 0"/></joint>'
    ))
    robot.append(E.fromstring(
        '<link name="camera_link">'
        '<inertial><origin xyz="0 -0.0175 0"/><mass value="0.072"/>'
        '<inertia ixx="0.00005" iyy="0.00001" izz="0.00005" ixy="0" ixz="0" iyz="0"/></inertial>'
        '<visual name="d435_body"><origin xyz="-0.0106 -0.0175 0"/>'
        '<geometry><box size="0.02505 0.090 0.025"/></geometry>'
        '<material name="d435_aluminum"><color rgba="0.72 0.74 0.77 1"/></material></visual>'
        '<visual name="d435_face"><origin xyz="0.0022 -0.0175 0"/>'
        '<geometry><box size="0.002 0.086 0.021"/></geometry>'
        '<material name="d435_glass"><color rgba="0.05 0.06 0.07 1"/></material></visual>'
        '<visual name="d435_lens_infra1"><origin xyz="0.0028 0.0 0" rpy="0 1.5707963267948966 0"/>'
        '<geometry><cylinder radius="0.0055" length="0.002"/></geometry>'
        '<material name="d435_lens"><color rgba="0.02 0.04 0.08 1"/></material></visual>'
        '<visual name="d435_lens_infra2"><origin xyz="0.0028 -0.050 0" rpy="0 1.5707963267948966 0"/>'
        '<geometry><cylinder radius="0.0055" length="0.002"/></geometry>'
        '<material name="d435_lens"/></visual>'
        '<visual name="d435_lens_color"><origin xyz="0.0028 0.015 0" rpy="0 1.5707963267948966 0"/>'
        '<geometry><cylinder radius="0.0055" length="0.002"/></geometry>'
        '<material name="d435_lens"/></visual>'
        '<collision name="d435_collision"><origin xyz="-0.0106 -0.0175 0"/>'
        '<geometry><box size="0.02505 0.090 0.025"/></geometry></collision>'
        '</link>'
    ))
    robot.append(E.fromstring(
        '<joint name="camera_link_joint" type="fixed"><parent link="camera_bottom_screw_frame"/>'
        '<child link="camera_link"/><origin xyz="0.0106 0.0175 0.0125" rpy="0 0 0"/></joint>'
    ))
    for sub, y_off in [('depth', 0.0), ('infra1', 0.0), ('infra2', -0.050), ('color', 0.015)]:
        frame = f'camera_{sub}_frame'
        optical = f'camera_{sub}_optical_frame'
        robot.append(E.fromstring(f'<link name="{frame}"/>'))
        robot.append(E.fromstring(
            f'<joint name="camera_{sub}_joint" type="fixed"><parent link="camera_link"/>'
            f'<child link="{frame}"/><origin xyz="0 {y_off} 0" rpy="0 0 0"/></joint>'
        ))
        robot.append(E.fromstring(f'<link name="{optical}"/>'))
        robot.append(E.fromstring(
            f'<joint name="camera_{sub}_optical_joint" type="fixed"><parent link="{frame}"/>'
            f'<child link="{optical}"/><origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/></joint>'
        ))

    robot.append(E.fromstring(
        '<gazebo reference="camera_depth_frame"><sensor name="realsense_depth_infra1" type="rgbd_camera">'
        '<always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>'
        '<topic>/gz_realsense/depth</topic><gz_frame_id>camera_depth_optical_frame</gz_frame_id>'
        '<camera><horizontal_fov>1.50098</horizontal_fov>'
        '<image><width>640</width><height>480</height><format>R8G8B8</format></image>'
        '<clip><near>0.15</near><far>100</far></clip>'
        '<camera_info_topic>/gz_realsense/depth/camera_info</camera_info_topic></camera>'
        '</sensor></gazebo>'
    ))
    robot.append(E.fromstring(
        '<gazebo reference="camera_infra2_frame"><sensor name="realsense_infra2" type="camera">'
        '<always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>'
        '<topic>/camera/infra2/image_raw</topic><gz_frame_id>camera_infra2_optical_frame</gz_frame_id>'
        '<camera><horizontal_fov>1.50098</horizontal_fov>'
        '<image><width>640</width><height>480</height><format>R8G8B8</format></image>'
        '<clip><near>0.15</near><far>100</far></clip>'
        '<camera_info_topic>/gz_realsense/infra2/camera_info</camera_info_topic></camera>'
        '</sensor></gazebo>'
    ))
    robot.append(E.fromstring(
        '<gazebo reference="camera_color_frame"><sensor name="realsense_color" type="camera">'
        '<always_on>true</always_on><update_rate>10</update_rate><visualize>false</visualize>'
        '<topic>/camera/color/image_raw</topic><gz_frame_id>camera_color_optical_frame</gz_frame_id>'
        '<camera><horizontal_fov>1.21126</horizontal_fov>'
        '<image><width>640</width><height>480</height><format>R8G8B8</format></image>'
        '<clip><near>0.15</near><far>100</far></clip>'
        '<camera_info_topic>/camera/color/camera_info</camera_info_topic></camera>'
        '</sensor></gazebo>'
    ))

    for plugin in robot.findall('gazebo/plugin'):
        if plugin.get('name') == 'gz::sim::systems::DiffDrive':
            plugin.find('topic').text = '/simulation/cmd_vel'
            E.SubElement(plugin, 'max_linear_velocity').text = '.8'
            E.SubElement(plugin, 'min_linear_velocity').text = '-.8'
            E.SubElement(plugin, 'max_angular_velocity').text = '1.5'
            E.SubElement(plugin, 'min_angular_velocity').text = '-1.5'

    g = E.SubElement(robot, 'gazebo')
    g.append(E.fromstring(
        '<plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">'
        '<odom_frame>world</odom_frame><robot_base_frame>base_footprint</robot_base_frame>'
        '<odom_topic>/simulation/ground_truth</odom_topic>'
        '<odom_publish_frequency>30</odom_publish_frequency><dimensions>3</dimensions></plugin>'
    ))

    for mesh in robot.findall('.//mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://leo_description/'):
            mesh.set('filename', str(leo / uri.removeprefix('package://leo_description/')))

    E.indent(robot)
    E.ElementTree(robot).write(OUT / 'leo_lidar.urdf', encoding='unicode', xml_declaration=True)
    r = subprocess.run(['gz', 'sdf', '-p', str(OUT / 'leo_lidar.urdf')], capture_output=True, text=True, check=True)
    (OUT / 'urdf_conversion.log').write_text(r.stderr)
    sdf = E.fromstring(r.stdout)
    model = sdf.find('model')
    model.set('name', 'leo_rover')
    E.SubElement(model, 'pose').text = f'{sx} {sy} {ground + 0.15} 0 0 0'
    for axis in model.findall('.//axis'):
        for mimic in axis.findall('mimic'):
            axis.remove(mimic)
    return model, leo


def main():
    meta = json.loads((ASSETS / 'terrain.json').read_text())
    z_dem = np.load(ASSETS / 'terrain.npy')
    sx, sy = float(meta['spawn_xyz'][0]), float(meta['spawn_xyz'][1])
    bx, by = sx + 4.5, sy + 7.5

    h_grid, h_max, g_entry_final, g_exit_final, tunnel_floor_z = build_rough_terrain(z_dem, sx, sy, bx, by)
    ground = ground_at(h_grid, sx, sy)
    aruco_model, aruco_info = build_aruco_markers(h_grid, g_entry_final, g_exit_final, tunnel_floor_z)
    marker_xy = [(m['world_xyz'][0], m['world_xyz'][1]) for m in aruco_info]
    rock_info = build_rockfield(h_grid, sx, sy, bx, by, marker_xy)
    rover_model, leo = build_leo_model(sx, sy, ground)

    cam_pose = f'{sx - 3.2} {sy - 3.2} {ground + 2.2} 0 0.42 0.7854'
    light_z = tunnel_floor_z + 2.02
    world = E.fromstring(f'''<sdf version="1.12"><world name="mdrs">
<physics name="rover_physics" type="ignored"><max_step_size>.002</max_step_size><real_time_factor>1</real_time_factor><dart><collision_detector>bullet</collision_detector></dart></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
<scene><ambient>.55 .55 .55 1</ambient><background>.55 .64 .72 1</background><shadows>false</shadows></scene>
<light name="sun" type="directional"><pose>0 0 1000 0 0 0</pose><diffuse>.75 .72 .65 1</diffuse><specular>.1 .1 .1 1</specular><direction>-.5 .3 -1</direction><cast_shadows>false</cast_shadows></light>
<light name="tunnel_light_1" type="point"><pose>50.5 {TUNNEL_Y} {light_z:.6f} 0 0 0</pose><diffuse>.85 .80 .70 1</diffuse><specular>.1 .1 .1 1</specular><attenuation><range>10.5</range><constant>0.4</constant><linear>0.25</linear><quadratic>0.05</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
<light name="tunnel_light_2" type="point"><pose>57.5 {TUNNEL_Y} {light_z:.6f} 0 0 0</pose><diffuse>.85 .80 .70 1</diffuse><specular>.1 .1 .1 1</specular><attenuation><range>10.5</range><constant>0.4</constant><linear>0.25</linear><quadratic>0.05</quadratic></attenuation><cast_shadows>false</cast_shadows></light>
<model name="mdrs_terrain"><static>true</static><link name="terrain">
<collision name="terrain_collision"><geometry><heightmap><uri>{OUT/'heightmap.png'}</uri><size>2001.953125 2001.953125 {h_max}</size><pos>0 0 0</pos><sampling>1</sampling></heightmap></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
<visual name="terrain_visual"><geometry><heightmap><uri>{OUT/'heightmap.png'}</uri><size>2001.953125 2001.953125 {h_max}</size><pos>0 0 0</pos><sampling>1</sampling><use_terrain_paging>false</use_terrain_paging><texture><diffuse>{OUT/'terrain_texture.png'}</diffuse><normal>{OUT/'flat_normal.png'}</normal><size>2000</size></texture></heightmap></geometry></visual>
</link></model>
<gui fullscreen="false"><plugin filename="MinimalScene" name="3D View"><gz-gui><title>Leo Rover - MDRS 2 km</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui><engine>ogre2</engine><scene>scene</scene><ambient_light>.55 .55 .55</ambient_light><background_color>.55 .64 .72</background_color><camera_pose>{cam_pose}</camera_pose></plugin><plugin filename="GzSceneManager" name="Scene Manager"/><plugin filename="InteractiveViewControl" name="View Control"/><plugin filename="CameraTracking" name="Camera Tracking"/></gui>
</world></sdf>''')

    world_el = world.find('world')
    world_el.append(rover_model)
    world_el.append(build_start_station(h_grid, sx, sy, bx, by))
    world_el.append(build_underground_tunnel(h_grid, g_entry_final, g_exit_final, tunnel_floor_z))
    world_el.append(aruco_model)
    world_el.append(build_rockfield_model())

    E.indent(world)
    E.ElementTree(world).write(OUT / 'mdrs.sdf', encoding='unicode', xml_declaration=True)

    gui_cfg = OUT / 'gui.config'
    if gui_cfg.exists():
        txt = gui_cfg.read_text()
        txt = re.sub(r'<camera_pose>[^<]*</camera_pose>', f'<camera_pose>{cam_pose}</camera_pose>', txt)
        gui_cfg.write_text(txt)

    (OUT / 'build_info.json').write_text(json.dumps({
        'terrain_size_m': [2000, 2000],
        'heightmap_pixels': [1025, 1025],
        'sdf_size_m': [2001.953125, 2001.953125],
        'collision_sample_span_m': [2000, 2000],
        'terrain_relief_m': h_max,
        'spawn_xyz': [sx, sy, ground + 0.15],
        'start_station_xyz': [bx, by, ground_at(h_grid, bx, by)],
        'underground_tunnel': {
            'center_y_m': TUNNEL_Y,
            'entry_ramp_x_m': [ENTRY_X0, BORE_X0],
            'bore_x_m': [BORE_X0, BORE_X1],
            'bore_length_m': BORE_X1 - BORE_X0,
            'exit_ramp_x_m': [BORE_X1, EXIT_X1],
            'inner_width_m': 2.0 * TUNNEL_INNER_HALF_W,
            'inner_height_m': TUNNEL_INNER_H,
            'entry_ground_z_m': g_entry_final,
            'exit_ground_z_m': g_exit_final,
            'floor_z_m': tunnel_floor_z,
            'start_markers': [m['id'] for m in aruco_info if m['role'] == 'tunnel_start'],
            'end_markers': [m['id'] for m in aruco_info if m['role'] == 'tunnel_end'],
        },
        'aruco_config': str(ARUCO_CONFIG),
        'aruco_markers': aruco_info,
        'rough_terrain': {
            'octaves': [[65, 0.55], [129, 0.32], [257, 0.18], [513, 0.09]],
            'near_craters': 12,
            'far_craters': 45,
            'mounds_and_ridges': 20,
            'rockfield': rock_info,
        },
        'source': 'installed official leo_description / leo_simulator',
        'leo_description_version': E.parse(leo / 'package.xml').findtext('version'),
        'urdf_sha256': hashlib.sha256((OUT / 'leo_lidar.urdf').read_bytes()).hexdigest(),
        'lidar': {
            'channels': 32,
            'horizontal_samples': 1024,
            'horizontal_fov_deg': 360,
            'vertical_min_deg': -30,
            'vertical_max_deg': 15,
            'range_m': [0.2, 80],
            'update_hz': 10,
            'frame': 'lidar_link',
            'topic': '/lidar/points',
        },
        'realsense_d435': {
            'model': 'Intel RealSense D435 (native Gazebo Jetty Ogre2 RGB-D + stereo)',
            'baseline_m': 0.050,
            'color_offset_y_m': 0.015,
            'resolution': [640, 480],
            'update_hz': 10,
            'topics': [
                '/camera/color/image_raw',
                '/camera/color/camera_info',
                '/camera/infra1/image_raw',
                '/camera/infra1/camera_info',
                '/camera/infra2/image_raw',
                '/camera/infra2/camera_info',
                '/camera/depth/image_raw',
                '/camera/depth/camera_info',
                '/camera/depth/color/points',
            ],
            'frames': {
                'mount_base': 'camera_link',
                'depth_optical': 'camera_depth_optical_frame',
                'infra1_optical': 'camera_infra1_optical_frame',
                'infra2_optical': 'camera_infra2_optical_frame',
                'color_optical': 'camera_color_optical_frame',
            },
        },
        'changes': [
            '32-channel 360-degree GPU lidar payload',
            'Intel RealSense D435 stereo / RGB-D camera payload (native Gazebo Jetty)',
            'procedural multi-scale rough terrain, craters, ridges, and 3-STL jagged rockfield',
            'flush start pad, 4 start-bay corner posts, and MDRS start outpost',
            'underground 14m concrete tunnel with entry/exit ramps and overburden hill',
            'configurable YAML-driven ArUco markers with variable sizes and dictionaries',
            'remove unsupported duplicate rocker mimic; retain official differential plugin',
            'command watchdog topic',
            'actual world pose publisher',
        ],
    }, indent=2))
    print(
        f'Built rough MDRS world ({rock_info["total_rocks"]} rocks), Start Station, '
        f'14m Underground Tunnel, and {len(aruco_info)} ArUco markers'
    )


if __name__ == '__main__':
    main()
