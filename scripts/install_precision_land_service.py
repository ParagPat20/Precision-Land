#!/usr/bin/env python3
"""
Backward-compatible wrapper.

This repo originally used `scripts/install_precision_land_service.py` for service installation.
It has been superseded by `scripts/pl_service.py`, which can also start/stop/status/logs and
manage the tmux session from one CLI.

This wrapper keeps existing docs and scripts working.
"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    pl_service = Path(__file__).resolve().parent / "pl_service.py"
    # Execute the new CLI as if it was run directly (argv stays intact).
    runpy.run_path(str(pl_service), run_name="__main__")


