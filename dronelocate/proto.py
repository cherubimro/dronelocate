"""Key expression scheme, message schemas, QoS policy.

Key expressions
---------------
    dc/{site}/{node}/evt/detect     detection events      interactive_high / block
    dc/{site}/{node}/iq             IQ queryable (pull)   background      / block
    dc/{site}/{node}/health         node telemetry        data_low        / drop
    dc/{site}/cmd/{node}            downlink retasking    interactive_high/ block
    dc/{site}/track/{tid}           fused tracks          data_high       / block
    dc/{site}/sim/truth             ground truth (SIM ONLY)

The sim/truth branch exists only so emulated nodes know what to render and so
the console can score the solver. Nothing downstream of the correlator reads
it. Delete that key expression and the system still localizes -- that is the
point of the demo.
"""

import json
import time
from dataclasses import dataclass, field

import cbor2

SCHEMA_VERSION = 3


def ke_detect(site, node="*"):
    return f"dc/{site}/{node}/evt/detect"


def ke_iq(site, node="*"):
    return f"dc/{site}/{node}/iq"


def ke_health(site, node="*"):
    return f"dc/{site}/{node}/health"


def ke_cmd(site, node="*"):
    return f"dc/{site}/cmd/{node}"


def ke_track(site, tid="*"):
    return f"dc/{site}/track/{tid}"


def ke_truth(site):
    return f"dc/{site}/sim/truth"


def encode(obj):
    return cbor2.dumps(obj)


def decode(buf):
    if hasattr(buf, "to_bytes"):
        buf = buf.to_bytes()
    return cbor2.loads(bytes(buf))


def detection_event(node_id, burst_id, t_utc_ns, fc_hz, fs_sps, bw_hz,
                    rssi_dbm, snr_db, cap_id, n_samples, fmt,
                    clock_offset_ns, clock_sigma_ns, lat, lon, alt_m,
                    classification="unknown", confidence=0.0, remote_id=None,
                    extra=None):
    """The primary product. Small, frequent, and the only thing that must
    never be dropped -- everything else is recoverable by asking again.

    extra carries source-specific provenance (UHD schedule error, GPSDO lock,
    per-node bearing) under keys the correlator does not read. Consumers must
    treat it as advisory: a node with no such hardware simply omits it.
    """
    ev = {
        "v": SCHEMA_VERSION,
        "node": node_id,
        "burst": int(burst_id),
        "t_utc_ns": int(t_utc_ns),
        "fc_hz": float(fc_hz),
        "fs_sps": float(fs_sps),
        "bw_hz": float(bw_hz),
        "rssi_dbm": float(rssi_dbm),
        "snr_db": float(snr_db),
        "cap_id": cap_id,
        "n_samples": int(n_samples),
        "fmt": fmt,
        # the field the whole solution depends on; mandatory, never optional
        "clk_off_ns": float(clock_offset_ns),
        "clk_sigma_ns": float(clock_sigma_ns),
        "ant": {"lat": lat, "lon": lon, "alt_m": alt_m},
        "cls": classification,
        "conf": float(confidence),
        "rid": remote_id,
    }
    if extra:
        ev["hw"] = dict(extra)
    return ev


def iq_metadata(node_id, cap_id, burst_id, t0_utc_ns, fs_sps, fc_hz, fmt,
                n_samples, clock_offset_ns):
    """Travels in the Zenoh attachment, not the payload. Relays can read and
    prioritise on it without touching the samples; if you add payload AEAD,
    this becomes the AAD so tampering with the timestamp breaks decryption."""
    return {
        "v": SCHEMA_VERSION,
        "node": node_id,
        "cap_id": cap_id,
        "burst": int(burst_id),
        "t0_utc_ns": int(t0_utc_ns),
        "fs_sps": float(fs_sps),
        "fc_hz": float(fc_hz),
        "fmt": fmt,
        "n_samples": int(n_samples),
        "clk_off_ns": float(clock_offset_ns),
    }


@dataclass
class SiteConfig:
    site: str
    origin: dict
    nodes: list = field(default_factory=list)
    radio: dict = field(default_factory=dict)
    sim: dict = field(default_factory=dict)
    solver: dict = field(default_factory=dict)
    hardware: dict = field(default_factory=dict)
    capture: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(site=d["site"], origin=d["origin"], nodes=d["nodes"],
                   radio=d.get("radio", {}), sim=d.get("sim", {}),
                   solver=d.get("solver", {}), hardware=d.get("hardware", {}),
                   capture=d.get("capture", {}))

    def node(self, node_id):
        for n in self.nodes:
            if n["id"] == node_id:
                return n
        raise KeyError(f"node {node_id} not in site config")


def now_ns():
    return time.time_ns()
