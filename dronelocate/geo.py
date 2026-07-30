"""WGS84 <-> local ENU tangent plane.

All filter and solver math runs in ENU metres. Lat/lon appears only at the
config boundary and at the display boundary. Doing least squares in degrees
introduces direction-dependent bias, so we never do it.
"""

import numpy as np

# WGS84
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)

C_LIGHT = 299792458.0


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sl, cl = np.sin(lat), np.cos(lat)
    n = _A / np.sqrt(1.0 - _E2 * sl * sl)
    x = (n + alt_m) * cl * np.cos(lon)
    y = (n + alt_m) * cl * np.sin(lon)
    z = (n * (1.0 - _E2) + alt_m) * sl
    return np.array([x, y, z], dtype=float)


def ecef_to_geodetic(x, y, z):
    """Bowring's method. Converges to sub-mm in one iteration at our altitudes."""
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - _E2))
    for _ in range(5):
        sl = np.sin(lat)
        n = _A / np.sqrt(1.0 - _E2 * sl * sl)
        alt = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1.0 - _E2 * n / (n + alt)))
    sl = np.sin(lat)
    n = _A / np.sqrt(1.0 - _E2 * sl * sl)
    alt = p / np.cos(lat) - n
    return np.degrees(lat), np.degrees(lon), alt


class LocalFrame:
    """East-North-Up tangent plane anchored at a site origin."""

    def __init__(self, lat_deg, lon_deg, alt_m=0.0):
        self.lat0 = float(lat_deg)
        self.lon0 = float(lon_deg)
        self.alt0 = float(alt_m)
        self._origin_ecef = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        lat, lon = np.radians(lat_deg), np.radians(lon_deg)
        sla, cla = np.sin(lat), np.cos(lat)
        slo, clo = np.sin(lon), np.cos(lon)
        # rows: east, north, up expressed in ECEF
        self._R = np.array([
            [-slo, clo, 0.0],
            [-sla * clo, -sla * slo, cla],
            [cla * clo, cla * slo, sla],
        ])

    @classmethod
    def from_dict(cls, d):
        """Accepts the site-config shape: {lat, lon, alt_m}."""
        return cls(d["lat"], d["lon"], d.get("alt_m", 0.0))

    def to_enu(self, lat_deg, lon_deg, alt_m):
        d = geodetic_to_ecef(lat_deg, lon_deg, alt_m) - self._origin_ecef
        return self._R @ d

    def to_geodetic(self, enu):
        enu = np.asarray(enu, dtype=float)
        ecef = self._R.T @ enu + self._origin_ecef
        return ecef_to_geodetic(*ecef)


def gdop(node_enu, target_enu, ref_index=0):
    """Geometric dilution of precision for a TDOA constellation.

    Returns (hdop, vdop, pdop). Large VDOP with ground-level nodes is the
    coplanarity problem: it is a property of geometry, not of the solver.
    """
    node_enu = np.asarray(node_enu, dtype=float)
    t = np.asarray(target_enu, dtype=float)
    d = node_enu - t
    r = np.linalg.norm(d, axis=1)
    u = d / r[:, None]  # unit vectors target -> node
    idx = [i for i in range(len(node_enu)) if i != ref_index]
    # TDOA measurement rows are differences of unit vectors
    h = np.array([u[ref_index] - u[i] for i in idx])
    try:
        cov = np.linalg.inv(h.T @ h)
    except np.linalg.LinAlgError:
        return np.inf, np.inf, np.inf
    dv = np.clip(np.diag(cov), 0.0, None)
    return float(np.sqrt(dv[0] + dv[1])), float(np.sqrt(dv[2])), float(np.sqrt(dv.sum()))
