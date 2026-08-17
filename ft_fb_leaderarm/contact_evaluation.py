#!/usr/bin/env python3
"""Evaluate ContactObservation states against independent contact intervals."""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

from .feedback_authorization import file_sha256


SCHEMA_VERSION = 1
ANALYSIS_TYPE = "physical_ft_contact_ground_truth_v1"
OBSERVATION_COLUMNS = {
    "t_s", "observer_contact_state", "observer_valid", "observer_model_ready"
}
GROUND_TRUTH_COLUMNS = {"start_s", "end_s"}


def _number(row, key, source):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"{source}: invalid {key}") from exc
    if not math.isfinite(value):
        raise RuntimeError(f"{source}: non-finite {key}")
    return value


def _binary(row, key, source):
    value = _number(row, key, source)
    if value not in (0.0, 1.0):
        raise RuntimeError(f"{source}: {key} must be 0 or 1")
    return value == 1.0


def _read_csv(path, required):
    target = Path(path).expanduser().resolve()
    if not target.is_file() or target.stat().st_size <= 0:
        raise RuntimeError(f"CSV is missing or empty: {target}")
    with target.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(required.difference(reader.fieldnames or ()))
        if missing:
            raise RuntimeError(f"{target}: missing CSV columns: {missing}")
        rows = list(reader)
    if not rows:
        raise RuntimeError(f"CSV has no rows: {target}")
    return target, rows


def _load_observations(path):
    target, rows = _read_csv(path, OBSERVATION_COLUMNS)
    observations = [
        {
            "t_s": _number(row, "t_s", target),
            "contact": _binary(row, "observer_contact_state", target),
            "valid": _binary(row, "observer_valid", target),
            "ready": _binary(row, "observer_model_ready", target),
        }
        for row in rows
    ]
    times = [row["t_s"] for row in observations]
    if len(times) < 2 or any(right <= left for left, right in zip(times, times[1:])):
        raise RuntimeError(f"{target}: observation timestamps must increase")
    return target, observations


def _load_intervals(path, first_time, last_time):
    target, rows = _read_csv(path, GROUND_TRUTH_COLUMNS)
    intervals = []
    for row in rows:
        start = _number(row, "start_s", target)
        end = _number(row, "end_s", target)
        if not first_time <= start < end <= last_time:
            raise RuntimeError(
                f"{target}: intervals must be positive and inside observations"
            )
        if intervals and start <= intervals[-1][1]:
            raise RuntimeError(
                f"{target}: intervals overlap, touch, or are unordered"
            )
        intervals.append((start, end))
    return target, intervals


def _validate_limits(min_precision, min_recall, max_onset_ms, max_release_ms):
    values = (min_precision, min_recall, max_onset_ms, max_release_ms)
    if any(not math.isfinite(value) for value in values):
        raise RuntimeError("contact evaluation limits must be finite")
    if not 0.0 < min_precision <= 1.0 or not 0.0 < min_recall <= 1.0:
        raise RuntimeError("minimum precision and recall must be within (0, 1]")
    if max_onset_ms < 0.0 or max_release_ms < 0.0:
        raise RuntimeError("latency limits must be non-negative")


def analyze_contact_evidence(
    observation_csv,
    ground_truth_csv,
    min_precision,
    min_recall,
    max_onset_latency_ms,
    max_release_latency_ms,
):
    """Return sample and event metrics without inventing acceptance limits."""
    limits = tuple(
        float(value) for value in (
            min_precision,
            min_recall,
            max_onset_latency_ms,
            max_release_latency_ms,
        )
    )
    _validate_limits(*limits)
    min_precision, min_recall, max_onset_ms, max_release_ms = limits
    observation_path, rows = _load_observations(observation_csv)
    truth_path, intervals = _load_intervals(
        ground_truth_csv, rows[0]["t_s"], rows[-1]["t_s"]
    )
    times = [row["t_s"] for row in rows]
    predicted = [row["contact"] for row in rows]
    truth = [
        any(start <= stamp < end for start, end in intervals)
        for stamp in times
    ]
    tp = sum(actual and expected for actual, expected in zip(predicted, truth))
    fp = sum(actual and not expected for actual, expected in zip(predicted, truth))
    fn = sum(not actual and expected for actual, expected in zip(predicted, truth))
    tn = len(rows) - tp - fp - fn
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / (tp + fn) if tp + fn else 0.0
    rising = [
        index for index, value in enumerate(predicted)
        if value and (index == 0 or not predicted[index - 1])
    ]
    falling = [
        index for index, value in enumerate(predicted)
        if not value and index > 0 and predicted[index - 1]
    ]
    events = []
    for index, (start, end) in enumerate(intervals):
        next_start = intervals[index + 1][0] if index + 1 < len(intervals) else math.inf
        onset = next((times[row] for row in rising if start <= times[row] < end), None)
        release = next((times[row] for row in falling if end <= times[row] < next_start), None)
        events.append(
            {
                "start_s": start,
                "end_s": end,
                "onset_latency_ms": None if onset is None else 1000.0 * (onset - start),
                "release_latency_ms": None if release is None else 1000.0 * (release - end),
            }
        )
    onset_latencies = [
        row["onset_latency_ms"] for row in events
        if row["onset_latency_ms"] is not None
    ]
    release_latencies = [
        row["release_latency_ms"] for row in events
        if row["release_latency_ms"] is not None
    ]
    onset_max = max(onset_latencies, default=None)
    release_max = max(release_latencies, default=None)
    invalid = sum(not row["valid"] for row in rows)
    not_ready = sum(not row["ready"] for row in rows)
    false_activations = sum(not truth[index] for index in rising)
    failures = []
    if invalid:
        failures.append("invalid observer samples exist")
    if not_ready:
        failures.append("model-not-ready observer samples exist")
    if not any(truth) or all(truth):
        failures.append("evidence must contain sampled CONTACT and FREE states")
    if fp:
        failures.append("FREE samples contain false CONTACT predictions")
    if precision < min_precision:
        failures.append("contact precision is below the required minimum")
    if recall < min_recall:
        failures.append("contact recall is below the required minimum")
    if len(onset_latencies) != len(intervals):
        failures.append("one or more ground-truth contact onsets were missed")
    elif onset_max > max_onset_ms:
        failures.append("maximum contact onset latency exceeded the limit")
    if len(release_latencies) != len(intervals):
        failures.append("one or more ground-truth contact releases were missed")
    elif release_max > max_release_ms:
        failures.append("maximum contact release latency exceeded the limit")

    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "ground_truth_contract": "independent_contact_intervals_same_clock",
        "sources": {
            "observation_csv": str(observation_path),
            "observation_sha256": file_sha256(observation_path),
            "ground_truth_csv": str(truth_path),
            "ground_truth_sha256": file_sha256(truth_path),
        },
        "limits": {
            "max_false_contact_samples": 0,
            "min_precision": min_precision,
            "min_recall": min_recall,
            "max_onset_latency_ms": max_onset_ms,
            "max_release_latency_ms": max_release_ms,
        },
        "metrics": {
            "samples": len(rows),
            "contact_events": len(intervals),
            "true_positive_samples": tp,
            "false_positive_samples": fp,
            "true_negative_samples": tn,
            "false_negative_samples": fn,
            "false_contact_activations": false_activations,
            "precision": precision,
            "recall": recall,
            "invalid_samples": invalid,
            "model_not_ready_samples": not_ready,
            "onset_latency_max_ms": onset_max,
            "release_latency_max_ms": release_max,
        },
        "events": events,
        "failures": failures,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Evaluate contact states against independent ground truth"
    )
    parser.add_argument("--observation-csv", required=True)
    parser.add_argument("--ground-truth-csv", required=True)
    parser.add_argument("--min-precision", type=float, required=True)
    parser.add_argument("--min-recall", type=float, required=True)
    parser.add_argument("--max-onset-latency-ms", type=float, required=True)
    parser.add_argument("--max-release-latency-ms", type=float, required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    try:
        args = parse_args(argv)
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite contact report: {output}")
        report = analyze_contact_evidence(
            args.observation_csv,
            args.ground_truth_csv,
            args.min_precision,
            args.min_recall,
            args.max_onset_latency_ms,
            args.max_release_latency_ms,
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(("PASS" if report["passed"] else "FAIL") + f": {output}")
        for failure in report["failures"]:
            print(f"- {failure}")
        return 0 if report["passed"] else 2
    except Exception as exc:
        print(f"ERROR: contact evaluation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
