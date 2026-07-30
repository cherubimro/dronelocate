"""Zenoh session configuration.

Star topology by default: the supernode listens, nodes connect as clients.
That matches the deployment discussed and it has a security property worth
keeping -- with a direct node-to-supernode link and no intermediate router,
TLS hop-by-hop encryption *is* end-to-end. Introduce a relay and it stops
being, at which point you want payload AEAD.

IPv4 endpoints are explicit because Zenoh's default listener is tcp/[::]:0,
which fails outright on IPv4-only hosts and containers.
"""

import json

import zenoh

DEFAULT_PORT = 7447


def _base():
    return {
        "scouting": {
            "multicast": {"enabled": False},
            "gossip": {"enabled": True},
        },
    }


def hub(port=DEFAULT_PORT, bind="0.0.0.0", tls=None):
    """Config for the process everything else connects to (the supernode)."""
    scheme = "tls" if tls else "tcp"
    c = _base()
    c["mode"] = "peer"
    c["listen"] = {"endpoints": [f"{scheme}/{bind}:{port}"]}
    if tls:
        c["transport"] = {"link": {"tls": {
            "root_ca_certificate": tls["ca"],
            "listen_private_key": tls["key"],
            "listen_certificate": tls["cert"],
            "enable_mtls": True,
            "close_link_on_expiration": True,
        }}}
    return zenoh.Config.from_json5(json.dumps(c))


def spoke(host="127.0.0.1", port=DEFAULT_PORT, tls=None, background_only=False):
    """Config for a sensor node.

    background_only opens a second link restricted to the low priority
    classes. Over TCP a large IQ transfer head-of-line blocks everything on
    the same connection; a separate link for bulk keeps detection events
    moving. QUIC gives this for free with independent streams, which is the
    main reason to prefer quic/ over tls/ in the field.
    """
    scheme = "tls" if tls else "tcp"
    ep = f"{scheme}/{host}:{port}"
    endpoints = [ep]
    if background_only:
        endpoints.append(f"{ep}?prio=6-7")   # data_low + background only

    c = _base()
    c["mode"] = "client"
    c["connect"] = {
        "endpoints": endpoints,
        "timeout_ms": -1,
        "retry": {"period_init_ms": 500, "period_max_ms": 4000,
                  "period_increase_factor": 2},
    }
    if tls:
        c["transport"] = {"link": {"tls": {
            "root_ca_certificate": tls["ca"],
            "enable_mtls": True,
            "connect_private_key": tls["key"],
            "connect_certificate": tls["cert"],
            "close_link_on_expiration": True,
        }}}
    return zenoh.Config.from_json5(json.dumps(c))


def add_args(ap):
    ap.add_argument("--hub", default="127.0.0.1",
                    help="supernode host for sensor nodes to connect to")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--tls-ca", default=None)
    ap.add_argument("--tls-cert", default=None)
    ap.add_argument("--tls-key", default=None)


def tls_from_args(a):
    if a.tls_ca and a.tls_cert and a.tls_key:
        return {"ca": a.tls_ca, "cert": a.tls_cert, "key": a.tls_key}
    return None
