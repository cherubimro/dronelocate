"""Where the vendored third-party trees live.

testradar (the Navico HALO toolkit: navico.py, the radar console's
webserver.py + viewer.html, and the recordings) is a git subtree under
``testradar/`` at the repository root -- AGPL-3.0-or-later, same authors,
pulled from dev.cs.upt.ro/alin.anton/testradar with ``git subtree pull``.
It is not pip-installed: the radar console is loaded by path from beside
whichever navico.py gets imported, so the import must resolve to a full
checkout, never to a bare module in site-packages.
"""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def testradar_dir():
    """The testradar checkout to use: $TESTRADAR if set, else the subtree."""
    return os.environ.get("TESTRADAR") or os.path.join(ROOT, "testradar")


def add_testradar_path():
    """Put the testradar checkout ahead of site-packages so ``import navico``
    finds the one with webserver.py and viewer.html beside it. Index 1: after
    the running script's own directory, before anything else."""
    d = testradar_dir()
    if os.path.isfile(os.path.join(d, "navico.py")) and d not in sys.path:
        sys.path.insert(1, d)
    return d
