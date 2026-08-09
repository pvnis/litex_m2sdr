#!/usr/bin/env python3
"""Use the frozen CP3303 controller with CP3310 endpoints and no 10 ms UL term."""

from __future__ import annotations

import importlib.util
from pathlib import Path


HERE = Path(__file__).resolve().parent
ART = HERE.parent
BASE = ART / "cp3303_extended_lifetime_no_rf_gate_20260730" / (
    "cp3303_execute_extended_lifetime_control.py"
)
ENDPOINTS = ART / "cp3310_preconnected_bidirectional_no_rf_gate_20260730"

spec = importlib.util.spec_from_file_location("cp3303_controller", BASE)
if spec is None or spec.loader is None:
    raise RuntimeError("failed to load frozen controller")
controller = importlib.util.module_from_spec(spec)
spec.loader.exec_module(controller)

controller.SERVER = ENDPOINTS / "cp3310_preconnected_tcp_server.py"
controller.CLIENT = ENDPOINTS / "cp3310_preconnected_tcp_client.py"
controller.NON_PRACH_EDGE_SCHEDULER_ERROR_SAMPLES = 0


if __name__ == "__main__":
    raise SystemExit(controller.main())
