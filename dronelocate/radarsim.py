"""Emulated radar plots. Transport-free so the same model drives the live
sim (radarnode.py --source sim, fed from dc/{site}/sim/truth) and the
offline fusion test (validate_fusion.py, fed from an analytic trajectory).

Allowed by the project's invariant the same way node.py --source sim is: an
emulated SENSOR may read the truth to decide what it would have measured,
and then publishes only what the real one would -- per-sector batches of
plots in the antenna frame, with per-spoke times, noise, missed detections
and false plots. Nothing on the solve or fusion path reads the truth.

Modelled on the HALO 20+ as measured on the bench (testradar README,
"Verified on hardware", and the project notes, "Radar node"):

- plots in the ANTENNA frame: true bearing = az + az_offset. The offset
  rendered here is the TRUE one; the supernode only knows the site
  config's value, and the two can be made to disagree.
- range cell = range_scale / 1024; horizontal beam 4.6 deg (the radar's
  own 0xC40A report).
- vertical fan +/-12.5 deg, good sensitivity only to ~9 deg: Pd inside
  the good part, pd * pd_edge_factor out to the edge, nothing beyond. Close
  and high is a blind cone.
- 16 batches per revolution, empty ones included.
- a revisit period that is NOT constant: rpm_alt / rpm_switch_s alternate
  two rates, the way the antenna follows whichever range transmits faster.
- Doppler flag from the radial speed against a threshold ("both" mode
  flags receding too).

What is NOT modelled, and matters on the real radar: target fluctuation and
RCS (Pd is a constant inside the fan), split or merged plots, clutter
structure (false plots are uniform Poisson), multipath off the ground, and
the radar's latency between measuring and stamping (latency_s, default 0;
plots advertise 50 ms of uncertainty as the real node does).
"""

import math

import numpy as np

SPOKES = 2048        # per revolution (navico.SPOKES)
PIXELS = 1024        # per spoke (navico.SPOKE_PIXELS)
SECTORS = 16

SIM_DEFAULTS = {
    "rpm": 24.0, "rpm_alt": None, "rpm_switch_s": 60.0,
    "range_scale_m": 3000.0, "max_det_range_m": 3000.0,
    "beam_h_deg": 4.6, "fan_good_deg": 9.0, "fan_max_deg": 12.5,
    "pd": 0.9, "pd_edge_factor": 0.3, "pd_bird": 0.8,
    "sig_r_cells": 1.0, "sig_az_deg": 0.5,
    # "matched": render each plot's azimuth noise with the sigma the plot
    # itself advertises (the detector's extent/sqrt(12), 0.7-1.5 deg), so
    # the fusion's covariance is right by construction. "fixed": render
    # sig_az_deg whatever is advertised -- the advertised sigma is then
    # 1.4-3x pessimistic, which is what the real detector may well be.
    "sig_az_mode": "matched",
    "false_per_rev": 2.0, "birds": 0,
    "doppler": "both", "dop_thresh_mps": 2.0,
    "latency_s": 0.0, "latency_sigma_s": 0.05,
    "az_offset_true_deg": 0.0,
    "label_plots": False,
}


class SimRadar:
    """Renders one sector at a time. Targets are (label, state_fn, pd)
    where state_fn(t) -> (pos_enu, vel_enu) or None; birds are internal."""

    def __init__(self, params, rid, origin_enu, seed=None):
        p = dict(SIM_DEFAULTS)
        p.update({k: v for k, v in (params or {}).items()
                  if not str(k).startswith("_") and v is not None})
        self.p = p
        self.rid = rid
        self.o = np.asarray(origin_enu, dtype=float)
        self.rng = np.random.default_rng(seed)
        self.next_id = 1
        self.birds = [self._bird() for _ in range(int(p["birds"]))]
        self.beam_spokes = p["beam_h_deg"] / 360.0 * SPOKES

    # --- clock ----------------------------------------------------------
    def rpm_at(self, t_rel):
        p = self.p
        if p.get("rpm_alt") and int(t_rel // p["rpm_switch_s"]) % 2 == 1:
            return float(p["rpm_alt"])
        return float(p["rpm"])

    # --- birds ----------------------------------------------------------
    def _bird(self):
        p = self.p
        r = p["range_scale_m"] * math.sqrt(self.rng.uniform(0.05, 0.8))
        th = self.rng.uniform(0, 2 * math.pi)
        hd = self.rng.uniform(0, 2 * math.pi)
        sp = self.rng.uniform(5.0, 15.0)
        return {"pos": self.o + np.array([r * math.sin(th), r * math.cos(th),
                                          self.rng.uniform(20, 80)]),
                "vel": np.array([sp * math.sin(hd), sp * math.cos(hd), 0.0]),
                "t": None}

    def step_birds(self, t):
        for b in self.birds:
            if b["t"] is None:
                b["t"] = t
                continue
            dt = t - b["t"]
            b["pos"] = b["pos"] + b["vel"] * dt
            b["t"] = t
            # meander: random heading change, stay inside the coverage
            ang = self.rng.normal(0.0, 0.3 * math.sqrt(max(dt, 1e-3)))
            c, s_ = math.cos(ang), math.sin(ang)
            v = b["vel"]
            b["vel"] = np.array([c * v[0] - s_ * v[1], s_ * v[0] + c * v[1], 0.0])
            if np.linalg.norm(b["pos"][:2] - self.o[:2]) > 0.9 * self.p["range_scale_m"]:
                b["vel"][:2] = -b["vel"][:2]

    # --- rendering ------------------------------------------------------
    def antenna_az(self, d):
        """Antenna-frame azimuth (deg) of an ENU offset, TRUE offset."""
        return (math.degrees(math.atan2(d[0], d[1]))
                - self.p["az_offset_true_deg"]) % 360.0

    def render(self, sector, t_start, t_end, targets=()):
        """Plots for one sector swept from t_start to t_end."""
        p = self.p
        a0 = sector * 360.0 / SECTORS
        span = 360.0 / SECTORS
        scale = p["range_scale_m"]
        cell = scale / PIXELS
        plots = []

        def crossing(state_fn):
            # When the beam centre crosses the target: the target moves a
            # fraction of a degree per sector sweep, so three fixed-point
            # steps converge to well under a spoke.
            tc = 0.5 * (t_start + t_end)
            for _ in range(3):
                st = state_fn(tc)
                if st is None:
                    return None
                rel = (self.antenna_az(st[0] - self.o) - a0) % 360.0
                if rel >= span:
                    return None
                tc = t_start + rel / span * (t_end - t_start)
            st = state_fn(tc)
            return None if st is None else (tc, st)

        allt = list(targets)
        for b in self.birds:
            allt.append(("bird", lambda t, b=b: None if b["t"] is None else
                         (b["pos"] + b["vel"] * (t - b["t"]), b["vel"]),
                         p["pd_bird"]))

        for tgt in allt:
            label, fn, pd = tgt[0], tgt[1], tgt[2]
            c = crossing(fn)
            if c is None:
                continue
            tc, (pos, vel) = c
            d = np.asarray(pos, dtype=float) - self.o
            r = float(np.linalg.norm(d))
            el = math.degrees(math.atan2(d[2], math.hypot(d[0], d[1])))
            if r < 2 * cell or r > min(scale, p["max_det_range_m"]):
                continue
            if abs(el) > p["fan_max_deg"]:
                continue
            pdet = pd if abs(el) <= p["fan_good_deg"] else pd * p["pd_edge_factor"]
            if self.rng.uniform() > pdet:
                continue
            vr = float(d @ np.asarray(vel, dtype=float)) / r
            shape = self._shape(cell)
            sa = shape["sig_az_deg"] if p["sig_az_mode"] == "matched" else p["sig_az_deg"]
            rng_m = r + self.rng.normal(0.0, p["sig_r_cells"] * cell)
            az = (self.antenna_az(d) + self.rng.normal(0.0, sa)) % 360.0
            dop = None
            if p["doppler"] != "off" and abs(vr) >= p["dop_thresh_mps"]:
                if vr < 0:
                    dop = "app"
                elif p["doppler"] == "both":
                    dop = "rec"
            plots.append(self._plot(tc + p["latency_s"], rng_m, az, cell, dop, label,
                                    shape))

        # false plots: Poisson per sector, uniform over the covered area
        lam = p["false_per_rev"] / SECTORS
        for _ in range(self.rng.poisson(lam)):
            rr = scale * math.sqrt(self.rng.uniform((50.0 / scale) ** 2, 1.0))
            az = a0 + self.rng.uniform(0, span)
            tc = t_start + (az - a0) / span * (t_end - t_start)
            plots.append(self._plot(tc, rr, az, cell, None, "false"))
        plots.sort(key=lambda q: q["t_utc_ns"])
        return plots

    def _shape(self, cell):
        """Plot extent and the sigmas radarnode's detector derives from it."""
        n_spokes = max(6, int(round(self.beam_spokes * self.rng.uniform(0.5, 1.1))))
        ext_bins = int(self.rng.integers(2, 5))
        ext_az = n_spokes * 360.0 / SPOKES
        ext_rng = ext_bins * cell
        return {"n_spokes": n_spokes, "ext_bins": ext_bins, "ext_az": ext_az,
                "ext_rng": ext_rng,
                "sig_rng_m": max(cell, ext_rng / math.sqrt(12)),
                "sig_az_deg": max(0.3, ext_az / math.sqrt(12))}

    def _plot(self, t, rng_m, az, cell, dop, label, shape=None):
        """Field-for-field what radarnode's Detector.finalise emits, sigmas
        included, so the fusion sees the numbers it would in the field. In
        "matched" mode (default) the rendered azimuth noise is the advertised
        sigma; in "fixed" mode it is sig_az_deg and the advertised one is
        pessimistic."""
        p = self.p
        shape = shape or self._shape(cell)
        n_spokes, ext_bins = shape["n_spokes"], shape["ext_bins"]
        ext_az, ext_rng = shape["ext_az"], shape["ext_rng"]
        n_cells = n_spokes * ext_bins
        pl = {
            "id": self.next_id,
            "t_utc_ns": int(round(t * 1e9)),
            "t_sigma_ns": int(p["latency_sigma_s"] * 1e9),
            "rng_m": float(rng_m), "az_deg": float(az),
            "sig_rng_m": shape["sig_rng_m"],
            "sig_az_deg": shape["sig_az_deg"],
            "ext_rng_m": ext_rng, "ext_az_deg": ext_az,
            "n_cells": n_cells, "n_spokes": n_spokes,
            "peak": int(self.rng.integers(8, 15)), "sum": n_cells * 10,
            "clutter_mu": 0.0, "clutter_occ": 0.0,
            "dop": dop, "dop_frac": 0.6 if dop else 0.0,
        }
        if p.get("label_plots"):
            pl["sim_label"] = label           # scoring aid; fusion never reads it
        self.next_id += 1
        return pl

    def batch(self, sector, rev, t_end, plots, rpm, az_offset_cfg):
        """The bus message for one sector, shaped like radarnode's."""
        p = self.p
        return {"sector": sector, "rev": rev, "t_utc_ns": int(round(t_end * 1e9)),
                "range_scale_m": float(p["range_scale_m"]), "plots": plots,
                "v": 3, "node": self.rid, "scanner": "A",
                "t_rev_s": 60.0 / rpm, "cell_m": p["range_scale_m"] / PIXELS,
                "az_ref": "antenna", "az_offset_deg": az_offset_cfg,
                "settings": {"use_mode": "custom", "gain": None,
                             "doppler": p["doppler"],
                             "doppler_speed_m_s": p["dop_thresh_mps"],
                             "noise_rejection": None, "scan_speed": None},
                "rpm_measured": rpm, "source": "sim"}

    def health(self, t, status, rpm, rev, plots_total, dropped=0):
        return {"v": 3, "node": self.rid, "kind": "radar",
                "scanner": "A", "t_utc_ns": int(t * 1e9),
                "status": status, "rpm": rpm, "clock_offset_ms": 0.0,
                "antenna_rpm_reported": rpm if status == "transmit" else None,
                "frames": 0, "seq_drops": 0, "seq_resets": 0,
                "range_scale_m": float(self.p["range_scale_m"]), "rev": rev,
                "maps": 1, "warm": True, "last_rev": {},
                "plots_total": plots_total, "ms_per_frame": 0.0,
                "publish_dropped": dropped, "nav": None, "source": "sim"}
