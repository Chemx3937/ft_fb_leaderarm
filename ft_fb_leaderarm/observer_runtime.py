#!/usr/bin/env python3
"""Measure the FS-05 runtime and FS-06 free-space residual gates."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from contact_observer_msgs.msg import ContactObservation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .contract import SAMPLE_HZ


SCHEMA_VERSION = 2
ANALYSIS_TYPE = "physical_ft_observer_runtime_v2"
MAX_FREE_FORCE_N = 1.0
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


def _observation_metrics(observations, start_sequence, end_sequence):
    if not isinstance(observations, list):
        raise RuntimeError("observations must be a list")
    selected = []
    for row in observations:
        if not isinstance(row, dict):
            raise RuntimeError("observation entry must be an object")
        sequence = _count(row, "prediction_sequence")
        if start_sequence < sequence <= end_sequence:
            selected.append((sequence, row))

    sequences = [sequence for sequence, _ in selected]
    expected_sequences = list(range(start_sequence + 1, end_sequence + 1))
    invalid = 0
    model_not_ready = 0
    contacts = 0
    force_norms = []
    frames = set()
    for _, row in selected:
        invalid += row.get("valid") is not True
        model_not_ready += row.get("model_ready") is not True
        state = _count(row, "contact_state")
        if state not in (0, 1):
            raise RuntimeError("observation contact_state must be FREE=0 or CONTACT=1")
        contacts += state == 1
        wrench = row.get("contact_wrench")
        if not isinstance(wrench, (list, tuple)) or len(wrench) != 6:
            raise RuntimeError("observation contact_wrench must contain six values")
        force = []
        for value in wrench[:3]:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise RuntimeError("observation contact_wrench must be numeric")
            value = float(value)
            if not math.isfinite(value):
                raise RuntimeError("observation contact_wrench must be finite")
            force.append(value)
        force_norms.append(math.sqrt(sum(value * value for value in force)))
        frames.add(str(row.get("frame_id", "")))

    return {
        "observation_samples": len(selected),
        "expected_observation_samples": len(expected_sequences),
        "observation_sequence_complete": sequences == expected_sequences,
        "invalid_observations": invalid,
        "model_not_ready_observations": model_not_ready,
        "contact_observations": contacts,
        "free_residual_force_norm_max_n": (
            max(force_norms) if force_norms else None
        ),
        "observation_frames": sorted(frames),
    }


def analyze_observer_runtime(start, end, observations):
    """Return a fail-closed FS-05/FS-06 report for one ready interval."""
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

    observation_metrics = _observation_metrics(
        observations,
        _count(start, "valid_predictions"),
        _count(end, "valid_predictions"),
    )
    actual_hz = deltas["valid_predictions"] / duration_s
    stale_publications = sum(reason_deltas.get(key, 0) for key in STALE_REASONS)
    runtime_failures = []
    if actual_hz < SAMPLE_HZ:
        runtime_failures.append(f"valid publish rate is below {SAMPLE_HZ} Hz")
    if deltas["invalid_publications"]:
        runtime_failures.append("invalid publications exist")
    if stale_publications:
        runtime_failures.append("stale publications exist")
    if deltas["deadline_misses"]:
        runtime_failures.append("deadline misses exist")
    if deltas["inference_failures"]:
        runtime_failures.append("inference failures exist")
    if deltas["cycles"] != (
        deltas["valid_predictions"] + deltas["invalid_publications"]
    ):
        runtime_failures.append("observer cycle accounting is inconsistent")

    free_failures = []
    if not observation_metrics["observation_sequence_complete"]:
        free_failures.append("ContactObservation sequence is incomplete")
    if observation_metrics["invalid_observations"]:
        free_failures.append("invalid ContactObservation samples exist")
    if observation_metrics["model_not_ready_observations"]:
        free_failures.append("model-not-ready ContactObservation samples exist")
    if observation_metrics["contact_observations"]:
        free_failures.append("CONTACT samples exist in the FREE interval")
    force_max = observation_metrics["free_residual_force_norm_max_n"]
    if force_max is None or force_max > MAX_FREE_FORCE_N:
        free_failures.append(
            f"FREE residual force exceeds {MAX_FREE_FORCE_N} N"
        )
    frames = observation_metrics["observation_frames"]
    if len(frames) != 1 or not frames[0]:
        free_failures.append("ContactObservation frame is missing or changed")

    failures = runtime_failures + free_failures

    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "binding": binding,
        "limits": {
            "required_hz": SAMPLE_HZ,
            "max_free_residual_force_n": MAX_FREE_FORCE_N,
            "max_contact_observations": 0,
            "max_invalid_publications": 0,
            "max_stale_publications": 0,
            "max_deadline_misses": 0,
            "max_inference_failures": 0,
        },
        "gates": {
            "FS-05": {
                "passed": not runtime_failures,
                "failures": runtime_failures,
            },
            "FS-06": {
                "passed": not free_failures,
                "failures": free_failures,
            },
        },
        "metrics": {
            "duration_s": duration_s,
            "valid_publish_hz": actual_hz,
            **deltas,
            "stale_publications": stale_publications,
            "invalid_reason_counts": reason_deltas,
            **observation_metrics,
        },
        "failures": failures,
    }


class ObserverRuntimeSampler(Node):
    def __init__(self, diagnostics_topic, observation_topic, duration_s):
        super().__init__("ft_observer_runtime_evaluator")
        self.duration_s = duration_s
        self.start = None
        self.end = None
        self.end_candidate = None
        self.observations = []
        self.error = None
        self.create_subscription(
            String, diagnostics_topic, self._diagnostics_callback, 10
        )
        self.create_subscription(
            ContactObservation,
            observation_topic,
            self._observation_callback,
            qos_profile_sensor_data,
        )

    def _diagnostics_callback(self, message):
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
            if self.end_candidate is None and _number(
                document, "uptime_s"
            ) - _number(
                self.start, "uptime_s"
            ) >= self.duration_s:
                self.end_candidate = document
                self._finish_if_complete()
        except Exception as exc:
            self.error = str(exc)

    def _observation_callback(self, message):
        if self.start is None or self.end is not None:
            return
        try:
            self.observations.append(
                {
                    "prediction_sequence": int(message.prediction_sequence),
                    "contact_wrench": list(message.contact_wrench),
                    "contact_state": int(message.contact_state),
                    "valid": bool(message.valid),
                    "model_ready": bool(message.model_ready),
                    "frame_id": str(message.header.frame_id),
                }
            )
            self._finish_if_complete()
        except Exception as exc:
            self.error = str(exc)

    def _finish_if_complete(self):
        if self.end_candidate is None:
            return
        target = _count(self.end_candidate, "valid_predictions")
        latest = max(
            (
                _count(row, "prediction_sequence")
                for row in self.observations
            ),
            default=_count(self.start, "valid_predictions"),
        )
        if latest >= target:
            self.end = self.end_candidate


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Measure the FS-05 runtime and FS-06 FREE acceptance gates"
    )
    parser.add_argument(
        "--diagnostics-topic", default="/ft_contact_observer/diagnostics"
    )
    parser.add_argument(
        "--observation-topic", default="/contact_observer/right/observation"
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
        node = ObserverRuntimeSampler(
            args.diagnostics_topic,
            args.observation_topic,
            args.duration_s,
        )
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

        report = analyze_observer_runtime(
            node.start, node.end, node.observations
        )
        report["source"] = {
            "diagnostics_topic": args.diagnostics_topic,
            "observation_topic": args.observation_topic,
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
