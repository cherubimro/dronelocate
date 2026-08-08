# Baked building footprints — OpenStreetMap data

**The `.json` files in this directory are derived from OpenStreetMap.**

- Source data © OpenStreetMap contributors
- Licensed under the **Open Database License (ODbL) v1.0**
- <https://www.openstreetmap.org/copyright>
- <https://opendatacommons.org/licenses/odbl/1-0/>

These files are a *Derived Database* under the ODbL. Anyone redistributing
them, or anything produced from them, must keep them under the ODbL and
credit OpenStreetMap contributors. That credit is shown to users: the
console displays "© OpenStreetMap contributors" in its legend whenever the
map or building layer is visible.

Note the licence boundary. The rest of this repository is AGPLv3, which
covers the *code*. It does not cover this directory's contents, which stay
under ODbL. Keeping them in their own directory with this notice is what
makes that separation legible rather than implied.

## Why these are committed

The console can fetch footprints live from the public Overpass API, and that
path still exists as a fallback. It is not dependable: Overpass is a free,
heavily loaded service that returns 504 under load and 429 if you ask
quickly, and during development it regularly refused whole cities, leaving
half-built skylines. Shipping the baked files makes the 3D layer work
immediately, offline, with no API quota and no partial cities — and restores
the offline-first property the rest of the console is built around.

They are small because they are not raw OSM: Timișoara is 1.5 MB here
against 26 MB of source JSON, roughly 17× smaller, because everything except
the geometry has been discarded.

## Format

One JSON object per city:

```json
{"lat": 45.7489, "lon": 21.2087, "span": 5000,
 "attribution": "© OpenStreetMap contributors (ODbL)",
 "b": [[height_m, x0,y0, x1,y1, ...], ...]}
```

Each entry in `b` is one building: its height in metres, followed by the
footprint ring as East/North pairs in **metres relative to that city's
centre**, at decimetre precision. The projection is the same flat
tangent-plane approximation the console uses — deliberately *not* the
solver's ellipsoidal transform, because the two must agree or switching
between cached and live data would visibly shift the city.

Filenames are `b_<lat>_<lon>_<span>.json` to four decimal places, which is
how the console finds the right file for the selected city and span. Change
`BLD_SPAN_M` in the console and these no longer match — regenerate them.

## Regenerating

```bash
python3.11 tools/fetch_buildings.py --city cluj      # one city
python3.11 tools/fetch_buildings.py --all            # all eight, ~30 min
```

Expect the odd square to fail; the tool reports how many and keeps the rest.
Heights are mostly synthetic — only about 1–6% of OSM footprints carry a
surveyed `height` or `building:levels`, so the remainder default to 9 m.
This is a plausible skyline, not survey data, and nothing in the solve path
reads it.
