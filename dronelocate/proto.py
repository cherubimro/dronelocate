"""Key expression scheme, message schemas, QoS policy.

Key expressions
---------------
    dc/{site}/{node}/evt/detect     detection events      interactive_high / block
    dc/{site}/{node}/iq             IQ queryable (pull)   background      / block
    dc/{site}/{node}/health         node telemetry        data_low        / drop
    dc/{site}/{node}/spectrum       PSD frame, display    data_low        / drop
    dc/{site}/{radar}/evt/plot      radar plot batches    interactive_high / block
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


def ke_spectrum(site, node="*"):
    return f"dc/{site}/{node}/spectrum"


def ke_plot(site, node="*"):
    """Radar plot batches (radarnode.py): one per 22.5 deg sector, empty
    sectors included. The radar's counterpart of ke_detect."""
    return f"dc/{site}/{node}/evt/plot"


def ke_cmd(site, node="*"):
    return f"dc/{site}/cmd/{node}"


def ke_track(site, tid="*"):
    return f"dc/{site}/track/{tid}"


def ke_dwell(site):
    return f"dc/{site}/dwell"


def ke_truth(site):
    return f"dc/{site}/sim/truth"


#: Wire formats, cheapest last. Bytes per complex sample.
WIRE_BYTES = {"cf32": 8.0, "ci8": 2.0, "ci2": 0.5}


def fmt_for_uplink(fs_sps, snippet_s, rate_hz, uplink_mbps, floor="ci2"):
    """Widest wire format whose sustained uplink fits this node's link.

    A node knows its own backhaul; the supernode does not need to. Each node
    therefore picks its own quantisation and states it in the event, which
    the correlator already honours per node -- so a fibre node can ship cf32
    and an LTE node ci2 for the SAME dwell, and the solver simply
    dequantises each accordingly.

    That asymmetry is the point of having the knob. At 56 MHz a 10 ms
    snippet at 2 bursts/s is 17.9 Mbit/s in ci8 and 4.5 in ci2: the first
    does not fit a ~10 Mbit/s cellular uplink and the second does, at about
    0.55 dB of SNR. On fibre neither number matters and you keep the bits.
    """
    if not uplink_mbps or uplink_mbps <= 0:
        return floor
    n = float(fs_sps) * float(snippet_s)
    for fmt in ("cf32", "ci8", "ci2"):
        if n * WIRE_BYTES[fmt] * 8.0 * float(rate_hz) / 1e6 <= float(uplink_mbps):
            return fmt
    return floor


def dwell_at(plan, dwell_s, t_s):
    """Which channel the whole fleet is on at time t, and the dwell index.

    Derived from UTC and a shared plan, exactly like UhdNodeSource.next_slot
    derives capture instants -- so ten independently started nodes agree on
    the frequency without any message arriving, and a node that misses a
    broadcast is merely out of date rather than lost. The broadcast on
    dc/{site}/dwell then does the job it is actually good at: changing the
    plan at runtime.

    Returns (index, channel_dict). An empty plan means "stare", i.e. the
    node keeps whatever its config gave it.
    """
    if not plan:
        return 0, None
    step = max(float(dwell_s), 1e-3)
    idx = int(t_s // step)
    return idx, plan[idx % len(plan)]


def dwell_message(seq, t_start_utc_ns, fc_hz, fs_sps, bw_hz, dwell_s, plan=None):
    """Where the fleet listens next, and when.

    The half of scheduled capture that a shared UTC grid cannot supply.
    Nodes can already agree on *instants* without exchanging a message --
    next_slot() derives them from UTC alone -- but nothing tells them which
    frequency to be on, and a fleet that disagrees about that produces
    exactly nothing: TDOA correlates IQ between nodes, so a node listening
    at 5.8 GHz while its neighbour is at 2.4 contributes no usable pair, not
    a degraded one.

    `plan` carries the next few dwells so a node that misses one message
    still knows where to be, and so a radio with a slow LO has somewhere to
    read its next tune from rather than being told at the last moment.
    """
    return {
        "v": SCHEMA_VERSION,
        "seq": int(seq),
        "t_start_utc_ns": int(t_start_utc_ns),
        "fc_hz": float(fc_hz),
        "fs_sps": float(fs_sps),
        "bw_hz": float(bw_hz),
        "dwell_s": float(dwell_s),
        "plan": list(plan or []),
    }


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


#: Quantisation for spectrum frames. Fixed and global on purpose: the whole
#: point of the display is comparing nodes against each other, so a per-frame
#: autoscale -- which is what every spectrum analyser does for one trace --
#: would make the strongest and weakest sensor look identical.
#:
#: The window has to cover two populations at once, which is why it is this
#: wide. Emulated nodes work in arbitrary units where the emitter sits near
#: unit amplitude, so their bins land around 0 dB; a real radio reports
#: fractions of ADC full scale, and a TinyB210 on a quiet band measured an
#: RMS near 1e-3, i.e. roughly -60 dB before its noise floor is reached.
#: A window sized for one of them clips the other flat -- measured, the first
#: attempt at -160..0 dB pinned every emulated bin to 255 and produced a
#: dynamic range of 0.6 dB, which draws as a blank panel.
#: 204 dB across 256 steps is 0.8 dB per step, coarser than the noise but
#: finer than anything an operator reads off a waterfall.
SPEC_DB_REF = -170.0
SPEC_DB_STEP = 0.8


def spectrum_frame(node_id, burst_id, t_utc_ns, fc_hz, fs_sps, bins_u8,
                   rssi_dbm, snr_db, classification="unknown"):
    """One power spectrum, quantised to a byte per bin.

    Display data, and priced accordingly: 256 bins is 256 bytes against the
    ~47 kB of IQ the same capture holds, so every node can publish one every
    burst without competing with anything that matters. It rides its own key
    at data_low/drop -- a spectrum that arrives late is worthless, exactly
    like health, and unlike a detection event it is never worth blocking for.
    """
    return {
        "v": SCHEMA_VERSION,
        "node": node_id,
        "burst": int(burst_id),
        "t_utc_ns": int(t_utc_ns),
        "fc_hz": float(fc_hz),
        "fs_sps": float(fs_sps),
        "n_bins": int(len(bins_u8)),
        "db_ref": SPEC_DB_REF,
        "db_step": SPEC_DB_STEP,
        "bins": bytes(bins_u8),
        "rssi_dbm": float(rssi_dbm),
        "snr_db": float(snr_db),
        "cls": classification,
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
    dwell: dict = field(default_factory=dict)
    #: Radar sensors: id, enu, az_offset_deg (null until calibrated), beam.
    radars: list = field(default_factory=list)
    #: Track-layer fusion parameters (dronelocate/fusion.py DEFAULTS).
    fusion: dict = field(default_factory=dict)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(site=d["site"], origin=d["origin"], nodes=d["nodes"],
                   radio=d.get("radio", {}), sim=d.get("sim", {}),
                   solver=d.get("solver", {}), hardware=d.get("hardware", {}),
                   capture=d.get("capture", {}), dwell=d.get("dwell", {}),
                   radars=d.get("radars", []), fusion=d.get("fusion", {}))

    def radar(self, radar_id):
        for r in self.radars:
            if r.get("id") == radar_id:
                return r
        raise KeyError(f"radar {radar_id} not in site config")

    def node(self, node_id):
        for n in self.nodes:
            if n["id"] == node_id:
                return n
        raise KeyError(f"node {node_id} not in site config")


def now_ns():
    return time.time_ns()
