#!/usr/bin/env python3
"""Regenerate web/map-sat.jpg — the offline satellite mosaic for /map.

Fetches Esri World Imagery tiles over the property, detects the "Map data
not yet available" placeholder tiles by their near-uniform pixels (they
decode as valid JPEGs, so an HTTP check alone passes), and walks down the
zoom ladder until every tile in the box has real content. Stitches with
cv2 (already in the rover venv) and writes the mosaic plus map-sat.json,
whose tile-origin metadata is what web/map.html uses to place lat/lon on
the image. Run from the venv when imagery coverage improves or the area
of interest changes:

    ~/code/rover1-venv/bin/python scripts/fetch_map_imagery.py
"""

from __future__ import annotations

import json
import math
import urllib.request
from pathlib import Path

import cv2
import numpy as np

# Coverage box: union of the Lot 14 parcel and the sidewalk-test path,
# with roughly a 15 m margin all around.
LAT_S, LAT_N = 41.59025, 41.59102
LON_W, LON_E = -90.49030, -90.48942

URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
WEB_DIR = Path(__file__).resolve().parent.parent / "web"
PLACEHOLDER_STD = 12.0  # tile stddev below this = uniform gray placeholder


def tile_xy(lat: float, lon: float, z: int) -> tuple[float, float]:
    n = 2 ** z
    lr = math.radians(lat)
    return ((lon + 180) / 360 * n,
            (1 - math.log(math.tan(lr) + 1 / math.cos(lr)) / math.pi) / 2 * n)


def fetch_level(z: int):
    x0, y1 = tile_xy(LAT_S, LON_W, z)
    x1, y0 = tile_xy(LAT_N, LON_E, z)
    tx0, tx1, ty0, ty1 = int(x0), int(x1), int(y0), int(y1)
    tiles = {}
    for ty in range(ty0, ty1 + 1):
        for tx in range(tx0, tx1 + 1):
            req = urllib.request.Request(
                URL.format(z=z, y=ty, x=tx),
                headers={"User-Agent": "rover1-map-prep/1.0"})
            with urllib.request.urlopen(req, timeout=20) as r:
                img = cv2.imdecode(np.frombuffer(r.read(), np.uint8),
                                   cv2.IMREAD_COLOR)
            if img is None or img.std() < PLACEHOLDER_STD:
                return None
            tiles[(tx, ty)] = img
    return tiles, tx0, ty0, tx1, ty1


def main() -> int:
    for z in (20, 19, 18, 17):
        got = fetch_level(z)
        if got:
            break
        print(f"z={z}: placeholder/no imagery, stepping down")
    else:
        print("no real imagery at any zoom level")
        return 1

    tiles, tx0, ty0, tx1, ty1 = got
    w, h = (tx1 - tx0 + 1) * 256, (ty1 - ty0 + 1) * 256
    mosaic = np.zeros((h, w, 3), np.uint8)
    for (tx, ty), img in tiles.items():
        mosaic[(ty - ty0) * 256:(ty - ty0 + 1) * 256,
               (tx - tx0) * 256:(tx - tx0 + 1) * 256] = img
    cv2.imwrite(str(WEB_DIR / "map-sat.jpg"), mosaic,
                [cv2.IMWRITE_JPEG_QUALITY, 88])
    (WEB_DIR / "map-sat.json").write_text(json.dumps({
        "z": z, "tx0": tx0, "ty0": ty0, "width_px": w, "height_px": h,
        "attribution": "Imagery © Esri, Maxar, Earthstar Geographics",
    }))
    res = 156543.03392 * math.cos(math.radians((LAT_S + LAT_N) / 2)) / (2 ** z)
    print(f"wrote web/map-sat.jpg z={z} {w}x{h}px ({res:.3f} m/px)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
