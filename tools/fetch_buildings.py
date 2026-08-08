#!/usr/bin/env python3
"""Bake OpenStreetMap building footprints into a local cache for the console.

Why this exists. The console's 3D layer can pull footprints live from the
public Overpass API, and that path works, but it is slow and rude: a 5 km
box is several MB and tens of seconds, the service rate-limits and returns
504 under load, and parsing that much OSM JSON on the browser's main thread
competes with the twelve simulation processes for the same cores. On a
laptop running both, that was measured to take supernode solve time from
110 ms to 4864 ms and visibly detach the drawn track from truth.

Running this once per site writes a compact file the console loads instead:
no network, no quota, no OSM JSON to parse, and the offline-first property
the live path gives up is restored. Buildings are cosmetic either way --
nothing in the solve path reads them.

    python3.11 tools/fetch_buildings.py                     # site origin
    python3.11 tools/fetch_buildings.py --city cluj
    python3.11 tools/fetch_buildings.py --lat 45.75 --lon 21.21 --span 5000

Output: console/cache/b_<lat>_<lon>_<span>.json, which is gitignored --
it is derived data, and OpenStreetMap's own ODbL terms are better served by
pointing people at the source than by vendoring a copy into the repository.
Attribution stays in the console legend.
"""

import argparse
import json
import math
import os
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

R_EARTH = 6378137.0
ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
]
# Same eight the console offers, so a cache exists for whatever is picked.
CITIES = {
    "arad": (46.1866, 21.3123),
    "bucharest": (44.4268, 26.1025),
    "cluj": (46.7712, 23.6236),
    "craiova": (44.3302, 23.7949),
    "galati": (45.4353, 28.0080),
    "iasi": (47.1585, 27.6014),
    "oradea": (47.0465, 21.9189),
    "timisoara": (45.7489, 21.2087),
}


def ll_to_enu(lat, lon, lat0, lon0):
    """Flat tangent plane, matching the console exactly.

    Deliberately the same approximation the browser uses, not the solver's
    ellipsoidal transform: the cache must land where the live path would, or
    switching between them would shift the city.
    """
    k = math.pi / 180.0
    return ((lon - lon0) * k * R_EARTH * math.cos(lat0 * k),
            (lat - lat0) * k * R_EARTH)


def overpass(query, tries=4):
    """POST with backoff. 429 and 504 are routine on the free instances."""
    last = None
    for attempt in range(tries):
        for ep in ENDPOINTS:
            try:
                req = urllib.request.Request(
                    ep, data=urllib.parse.urlencode({"data": query}).encode(),
                    headers={"User-Agent": "dronelocate-fetch-buildings/1.0"})
                return json.loads(urllib.request.urlopen(req, timeout=180).read())
            except Exception as e:      # noqa: BLE001 - any failure is a retry
                last = e
        time.sleep(2.5 * (attempt + 1))
    raise last


def fetch(lat, lon, span, grid=3, verbose=True):
    """Fetch in a grid of smaller boxes: one big query times out far more."""
    half = span / 2.0
    d_lat = half / (R_EARTH * math.pi / 180.0)
    d_lon = half / (R_EARTH * math.pi / 180.0 * math.cos(math.radians(lat)))

    boxes = []
    for i in range(grid):
        for j in range(grid):
            s = lat - d_lat + 2 * d_lat * i / grid
            n = lat - d_lat + 2 * d_lat * (i + 1) / grid
            w = lon - d_lon + 2 * d_lon * j / grid
            e = lon - d_lon + 2 * d_lon * (j + 1) / grid
            boxes.append((s, w, n, e))

    out, tagged, failed = [], 0, 0
    for idx, (s, w, n, e) in enumerate(boxes, 1):
        bbox = f"{s:.5f},{w:.5f},{n:.5f},{e:.5f}"
        try:
            data = overpass(
                f'[out:json][timeout:90];way["building"]({bbox});out geom;')
        except Exception as err:        # noqa: BLE001
            failed += 1
            if verbose:
                print(f"  [{idx}/{len(boxes)}] failed: {err}", file=sys.stderr)
            continue

        got = 0
        for way in data.get("elements", []):
            geom = way.get("geometry")
            if not geom or len(geom) < 4:
                continue
            tags = way.get("tags", {})
            try:
                h = float(str(tags["height"]).split()[0])
                is_tagged = True
            except (KeyError, ValueError, IndexError):
                is_tagged = False
                try:
                    h = float(tags["building:levels"]) * 3.2
                except (KeyError, ValueError):
                    h = 9.0
            # height=0 and absurd outliers both exist in OSM and both render
            # badly; clamp rather than trust the tag.
            h = max(3.0, min(h, 300.0))
            tagged += is_tagged

            ring = [ll_to_enu(p["lat"], p["lon"], lat, lon) for p in geom[:-1]]
            if len(ring) < 3:
                continue
            packed = [round(h, 1)]
            for x, y in ring:
                packed.append(round(x, 1))      # decimetre precision is far
                packed.append(round(y, 1))      # finer than a drawn building
            out.append(packed)
            got += 1
        if verbose:
            print(f"  [{idx}/{len(boxes)}] {got} buildings (total {len(out)})")
        time.sleep(1.2)                 # pace: the limiter rejects bursts
    return out, tagged, failed


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--city", choices=sorted(CITIES),
                    help="one of the console's cities")
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--span", type=int, default=5000,
                    help="box side in metres; must match the console's "
                         "BLD_SPAN_M or the cache will not be found")
    ap.add_argument("--config", default=os.path.join(here, "config/site.json"))
    ap.add_argument("--all", action="store_true", help="every city")
    a = ap.parse_args()

    targets = []
    if a.all:
        targets = [(n, *ll) for n, ll in sorted(CITIES.items())]
    elif a.city:
        targets = [(a.city, *CITIES[a.city])]
    elif a.lat is not None and a.lon is not None:
        targets = [("custom", a.lat, a.lon)]
    else:
        with open(a.config) as f:
            o = json.load(f)["origin"]
        targets = [("site origin", o["lat"], o["lon"])]

    outdir = os.path.join(here, "console", "cache")
    os.makedirs(outdir, exist_ok=True)

    for name, lat, lon in targets:
        print(f"{name}: {lat:.4f},{lon:.4f} span {a.span} m")
        t0 = time.time()
        buildings, tagged, failed = fetch(lat, lon, a.span)
        if not buildings:
            print("  nothing returned; leaving any existing cache alone",
                  file=sys.stderr)
            continue
        path = os.path.join(outdir, f"b_{lat:.4f}_{lon:.4f}_{a.span}.json")
        with open(path, "w") as f:
            json.dump({"lat": lat, "lon": lon, "span": a.span,
                       "attribution": "© OpenStreetMap contributors (ODbL)",
                       "b": buildings}, f, separators=(",", ":"))
        mb = os.path.getsize(path) / 1e6
        print(f"  {len(buildings)} buildings, {tagged} with surveyed height, "
              f"{failed} squares lost -> {os.path.relpath(path, here)} "
              f"({mb:.1f} MB, {time.time()-t0:.0f}s)")


if __name__ == "__main__":
    main()
