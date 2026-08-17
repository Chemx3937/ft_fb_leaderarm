#!/usr/bin/env python3
"""Measure the FS-05 observer runtime gate from diagnostics snapshots."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .contract import SAMPLE_HZ


SCHEMA_VERSION = 1
ANALYSIS_TYPE = "physical_ft_observer_runtime_v1"
COUNTERS = (
    "cycles",
    "valid_predictions",
    "invalid_publications",
    "deadline_misses",
    "inference_failures",
)
STALE_REASONS = ("stale_or_future_input", "locally_stale_input")
BINDINGS = (
    "model_sha256",
    "zero_set_id",
    "payload_id",
    "controller_config_hash",
)


def _number(document, key):
    value = document.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"diagnostics {key} must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise RuntimeError(f"diagnostics {key} must be finite")
    return value


def _count(document, key):
    value = _number(document, key)
    if value < 0.0 or value != int(value):
        raise RuntimeError(f"diagnostics {key} must be a non-negative integer")
    return int(value)


def _reason_counts(document):
    values = document.get("invalid_reason_counts")
    if not isinstance(values, dict):
        raise RuntimeError("diagnostics invalid_reason_counts must be an object")
    return {str(key): _count(values, key) for key in values}


def analyze_observer_runtime(start, end):
    """Return a fail-closed FS-05 report for two ready diagnostics snapshots."""
    if not isinstance(start, dict) or not isinstance(end, dict):
        raise RuntimeError("start and end diagnostics must be objects")
    for label, document in (("start", start), ("end", end)):
        if not all(
            document.get(key) is True
            for key in (
                "approved_model",
                "model_ready",
                "baseline_ready",
                "observer_ready",
            )
        ):
            raise RuntimeError(f"{label} diagnostics are not observer-ready")
        if abs(_number(document, "sample_hz") - SAMPLE_HZ) > 1.0e-9:
            raise RuntimeError(
                f"{label} diagnostics violate the {SAMPLE_HZ} Hz contract"
            )

    binding = {}
    for key in BINDINGS:
        value = start.get(key)
        if not isinstance(value, str) or not value:
            raise RuntimeError(f"start diagnostics {key} is missing")
        if end.get(key) != value:
            raise RuntimeError(f"diagnostics {key} changed during measurement")
        binding[key] = value

    duration_s = _number(end, "uptime_s") - _number(start, "uptime_s")
    if duration_s <= 0.0:
        raise RuntimeError("diagnostics measurement duration must be positive")

    deltas = {}
    for key in COUNTERS:
        deltas[key] = _count(end, key) - _count(start, key)
        if deltas[key] < 0:
            raise RuntimeError(f"diagnostics {key} moved backwards")

    start_reasons = _reason_counts(start)
    end_reasons = _reason_counts(end)
    reason_deltas = {}
    for key in sorted(set(start_reasons) | set(end_reasons)):
        delta = end_reasons.get(key, 0) - start_reasons.get(key, 0)
        if delta < 0:
            raise RuntimeError(
                f"diagnostics invalid reason {key!r} moved backwards"
            )
        if delta:
            reason_deltas[key] = delta

    actual_hz = deltas["valid_predictions"] / duration_s
    stale_publications = sum(reason_deltas.get(key, 0) for key in STALE_REASONS)
    failures = []
    if actual_hz < SAMPLE_HZ:
        failures.append(f"valid publish rate is below {SAMPLE_HZ} Hz")
    if deltas["invalid_publications"]:
        failures.append("invalid publications exist")
    if stale_publications:
        failures.append("stale publications exist")
    if deltas["deadline_misses"]:
        failures.append("deadline misses exist")
    if deltas["inference_failures"]:
        failures.append("inference failures exist")
    if deltas["cycles"] != (
        deltas["valid_predictions"] + deltas["invalid_publications"]
    ):
        failures.append("observer cycle accounting is inconsistent")

    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "binding": binding,
        "limits": {
            "required_hz": SAMPLE_HZ,
            "max_invalid_publications": 0,
            "max_stale_publications": 0,
            "max_deadline_misses": 0,
            "max_inference_failures": 0,
        },
        "metrics": {
            "duration_s": duration_s,
            "valid_publish_hz": actual_hz,
            **deltas,
            "stale_publications": stale_publications,
            "invalid_reason_counts": reason_deltas,
        },
        "failures": failures,
    }


class DiagnosticsSampler(Node):
    def __init__(self, topic, duration_s):
        super().__init__("ft_observer_runtime_evaluator")
        self.duration_s = duration_s
        self.start = None
        self.end = None
        self.error = None
        self.create_subscription(String, topic, self._callback, 10)

    def _callback(self, message):
        try:
            document = json.loads(message.data)
            if not isinstance(document, dict):
                raise RuntimeError("diagnostics JSON must be an object")
            if not all(
                document.get(key) is True
                for key in (
                    "approved_model",
                    "model_ready",
                    "baseline_ready",
                    "observer_ready",
                )
            ):
                return
            if self.start is None:
                self.start = document
                return
            if _number(document, "uptime_s") - _number(
                self.start, "uptime_s"
            ) >= self.duration_s:
                self.end = document
        except Exception as exc:
            self.error = str(exc)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Measure the FS-05 observer runtime acceptance gate"
    )
    parser.add_argument(
        "--diagnostics-topic", default="/ft_contact_observer/diagnostics"
    )
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--timeout-s", type=float)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    node = None
    try:
        args = parse_args(argv)
        if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
            raise RuntimeError("duration-s must be finite and positive")
        timeout_s = args.timeout_s
        if timeout_s is None:
            timeout_s = args.duration_s + 30.0
        if not math.isfinite(timeout_s) or timeout_s <= args.duration_s:
            raise RuntimeError("timeout-s must be finite and greater than duration-s")
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite runtime report: {output}")

        rclpy.init()
        node = DiagnosticsSampler(args.diagnostics_topic, args.duration_s)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and node.end is None and node.error is None:
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "timed out waiting for a ready diagnostics interval"
                )
            rclpy.spin_once(node, timeout_sec=0.25)
        if node.error:
            raise RuntimeError(node.error)
        if node.start is None or node.end is None:
            raise RuntimeError("observer runtime interval was not collected")

        report = analyze_observer_runtime(node.start, node.end)
        report["source"] = {
            "diagnostics_topic": args.diagnostics_topic,
            "requested_duration_s": args.duration_s,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(("PASS" if report["passed"] else "FAIL") + f": {output}")
        for failure in report["failures"]:
            print(f"- {failure}")
        return 0 if report["passed"] else 2
    except Exception as exc:
        print(f"ERROR: observer runtime evaluation failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
