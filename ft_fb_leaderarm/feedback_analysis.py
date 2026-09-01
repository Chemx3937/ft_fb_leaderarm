#!/usr/bin/env python3
"""Analyze immutable FT Observer/Leader CSV evidence for staged feedback."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

from .contract import (
    OPERATIONAL_FREE_FORCE_HARD_MAX_LIMIT_N,
    OPERATIONAL_FREE_FORCE_P95_LIMIT_N,
    OPERATIONAL_FREE_FORCE_P99_LIMIT_N,
    runtime_model_acceptance,
)

from .feedback_authorization import REFERENCE_CLIP_NM


ANALYSIS_SCHEMA_VERSION = 2
ANALYSIS_TYPE = "physical_ft_feedback_analysis_v2"
ONSET_ANALYSIS_TYPE = "feedback_onset_analysis_v1"
TARGET_TO_EVIDENCE_GAIN = {0.40: 0.0, 1.00: 0.40}
SAFETY_LIMITS = {
    "max_free_force_p95_n": OPERATIONAL_FREE_FORCE_P95_LIMIT_N,
    "max_free_force_p99_n": OPERATIONAL_FREE_FORCE_P99_LIMIT_N,
    "max_free_force_error_n": OPERATIONAL_FREE_FORCE_HARD_MAX_LIMIT_N,
    "max_pose_step_deg": 1.0,
    "max_velocity_reversal_hz": 8.0,
    "max_source_age_ms": 20.0,
    "max_record_gap_ms": 10.0,
    "min_run_duration_s": 10.0,
    "min_csv_hz": 250.0,
    "min_contact_activations": 3,
    "min_contact_feedback_nonzero_fraction": 0.95,
}
REQUIRED_COLUMNS = {
    "t_s",
    "dt_ms",
    "state",
    "feedback_gain_scale_contract",
    "observer_valid",
    "observer_model_ready",
    "observer_source_age_ms",
    "observer_contact_state",
    "observer_contact_score_N",
    "observer_prediction_age_ms",
    "observer_latency_ms",
    "observer_source_sequence",
    "observer_prediction_sequence",
    "fe_raw_fx",
    "fe_raw_fy",
    "fe_raw_fz",
    *(f"leader_j{index}_deg" for index in range(1, 7)),
    *(f"follower_j{index}_deg" for index in range(1, 7)),
    *(f"leader_dq_j{index}" for index in range(1, 7)),
    *(f"tau_fb_j{index}" for index in range(1, 7)),
}
ONSET_REQUIRED_COLUMNS = {
    "t_s",
    "state",
    "feedback_gain_scale_contract",
    "observer_valid",
    "observer_model_ready",
    "fe_age_ms",
    "observer_source_age_ms",
    "observer_contact_state",
    "contact_scale",
    *(f"tau_fb_j{index}" for index in range(1, 7)),
}


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_limits(limits):
    required = set(SAFETY_LIMITS) | {"max_contact_force_n"}
    if not isinstance(limits, dict) or set(limits) != required:
        raise RuntimeError(f"analysis limits must contain exactly {sorted(required)}")
    for key, value in limits.items():
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise RuntimeError(f"{key} must be finite and positive")
    if int(limits["min_contact_activations"]) != limits["min_contact_activations"]:
        raise RuntimeError("min_contact_activations must be an integer")
    for key in (
        "max_free_force_p95_n",
        "max_free_force_p99_n",
        "max_free_force_error_n",
        "max_pose_step_deg",
        "max_velocity_reversal_hz",
        "max_source_age_ms",
        "max_record_gap_ms",
    ):
        if limits[key] > SAFETY_LIMITS[key]:
            raise RuntimeError(f"{key} cannot exceed {SAFETY_LIMITS[key]}")
    for key in (
        "min_run_duration_s",
        "min_csv_hz",
        "min_contact_activations",
        "min_contact_feedback_nonzero_fraction",
    ):
        if limits[key] < SAFETY_LIMITS[key]:
            raise RuntimeError(f"{key} cannot be below {SAFETY_LIMITS[key]}")
    if limits["min_contact_feedback_nonzero_fraction"] > 1.0:
        raise RuntimeError("min_contact_feedback_nonzero_fraction cannot exceed 1.0")


def _number(row, key):
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"CSV contains invalid {key!r}") from exc
    if not math.isfinite(value):
        raise RuntimeError(f"CSV contains non-finite {key!r}")
    return value


def _activation_count(contact):
    contact = np.asarray(contact, dtype=np.bool_)
    return int(np.count_nonzero(contact & ~np.r_[False, contact[:-1]]))


def _velocity_reversal_rates(dq, duration_s, active_speed_rad_s=0.05):
    rates = []
    for joint in range(6):
        active = dq[:, joint]
        signs = np.sign(active[np.abs(active) >= active_speed_rad_s])
        reversals = int(np.count_nonzero(signs[1:] != signs[:-1]))
        rates.append(reversals / duration_s)
    return rates


def analyze_csv(path, expected_state):
    target = Path(path).expanduser().resolve()
    if not target.is_file() or target.stat().st_size <= 0:
        raise RuntimeError(f"evidence CSV is missing or empty: {target}")
    rows = []
    with target.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(REQUIRED_COLUMNS.difference(reader.fieldnames or ()))
        if missing:
            raise RuntimeError(f"{target}: CSV columns are missing: {missing}")
        rows = [row for row in reader if row.get("state", "").strip().upper() == "FAST"]
    if len(rows) < 2:
        raise RuntimeError(f"{target}: no usable FAST interval")

    times = np.asarray([_number(row, "t_s") for row in rows])
    dt_ms = np.asarray([_number(row, "dt_ms") for row in rows])
    if np.any(np.diff(times) <= 0.0):
        raise RuntimeError(f"{target}: FAST timestamps are not increasing")
    duration_s = float(times[-1] - times[0])
    if duration_s <= 0.0:
        raise RuntimeError(f"{target}: FAST duration is not positive")

    valid = np.asarray([_number(row, "observer_valid") == 1.0 for row in rows])
    gain_scale = np.asarray(
        [_number(row, "feedback_gain_scale_contract") for row in rows]
    )
    ready = np.asarray(
        [_number(row, "observer_model_ready") == 1.0 for row in rows]
    )
    source_age_ms = np.asarray(
        [_number(row, "observer_source_age_ms") for row in rows]
    )
    contact_values = np.asarray(
        [_number(row, "observer_contact_state") for row in rows]
    )
    if np.any(~np.isin(contact_values, (0.0, 1.0))):
        raise RuntimeError(f"{target}: observer_contact_state must be FREE=0 or CONTACT=1")
    contact = contact_values == 1.0
    score = np.asarray([_number(row, "observer_contact_score_N") for row in rows])
    prediction_age = np.asarray(
        [_number(row, "observer_prediction_age_ms") for row in rows]
    )
    latency = np.asarray([_number(row, "observer_latency_ms") for row in rows])
    source_sequence = np.asarray(
        [_number(row, "observer_source_sequence") for row in rows]
    )
    prediction_sequence = np.asarray(
        [_number(row, "observer_prediction_sequence") for row in rows]
    )
    if (
        np.any(source_sequence < 0.0)
        or np.any(prediction_sequence < 0.0)
        or np.any(source_sequence != np.floor(source_sequence))
        or np.any(prediction_sequence != np.floor(prediction_sequence))
        or np.any(np.diff(source_sequence) < 0.0)
        or np.any(np.diff(prediction_sequence) < 0.0)
    ):
        raise RuntimeError(f"{target}: observer sequence moved backwards")

    force = np.asarray(
        [[_number(row, f"fe_raw_f{axis}") for axis in ("x", "y", "z")] for row in rows]
    )
    force_norm = np.linalg.norm(force, axis=1)
    leader = np.asarray(
        [[_number(row, f"leader_j{index}_deg") for index in range(1, 7)] for row in rows]
    )
    follower = np.asarray(
        [[_number(row, f"follower_j{index}_deg") for index in range(1, 7)] for row in rows]
    )
    dq = np.asarray(
        [[_number(row, f"leader_dq_j{index}") for index in range(1, 7)] for row in rows]
    )
    feedback = np.asarray(
        [[_number(row, f"tau_fb_j{index}") for index in range(1, 7)] for row in rows]
    )
    reversal_rates = _velocity_reversal_rates(dq, duration_s)
    contact_feedback = np.any(np.abs(feedback[contact]) > 1.0e-6, axis=1)
    return {
        "path": str(target),
        "sha256": file_sha256(target),
        "size_bytes": target.stat().st_size,
        "expected_state": expected_state,
        "fast_rows": len(rows),
        "duration_s": duration_s,
        "actual_hz": (len(rows) - 1) / duration_s,
        "feedback_gain_scale_min": float(np.min(gain_scale)),
        "feedback_gain_scale_max": float(np.max(gain_scale)),
        "max_dt_ms": float(np.max(dt_ms[1:])),
        "invalid_samples": int(np.count_nonzero(~valid)),
        "model_not_ready_samples": int(np.count_nonzero(~ready)),
        "source_age_min_ms": float(np.min(source_age_ms)),
        "source_age_max_ms": float(np.max(source_age_ms)),
        "prediction_age_min_ms": float(np.min(prediction_age)),
        "prediction_age_max_ms": float(np.max(prediction_age)),
        "observer_latency_min_ms": float(np.min(latency)),
        "observer_latency_max_ms": float(np.max(latency)),
        "contact_activations": _activation_count(contact),
        "contact_fraction": float(np.mean(contact)),
        "force_norm_max_n": float(np.max(force_norm)),
        "force_norm_p95_n": float(np.percentile(force_norm, 95.0)),
        "force_norm_p99_n": float(np.percentile(force_norm, 99.0)),
        "score_force_norm_difference_max_n": float(np.max(np.abs(score - force_norm))),
        "leader_pose_step_max_deg": float(np.max(np.abs(np.diff(leader, axis=0)))),
        "follower_pose_step_max_deg": float(np.max(np.abs(np.diff(follower, axis=0)))),
        "leader_velocity_reversal_hz_by_joint": reversal_rates,
        "leader_velocity_reversal_hz_max": max(reversal_rates),
        "feedback_abs_max_nm_by_joint": np.max(np.abs(feedback), axis=0).tolist(),
        "feedback_abs_max_nm": float(np.max(np.abs(feedback))),
        "contact_feedback_nonzero_fraction": (
            float(np.mean(contact_feedback)) if len(contact_feedback) else 0.0
        ),
    }


def analyze_feedback_onsets(
    path,
    max_rise_time_ms,
    max_torque_step_nm,
    min_contact_activations=3,
    max_source_age_ms=20.0,
    max_record_gap_ms=10.0,
):
    limits = {
        "max_rise_time_ms": max_rise_time_ms,
        "max_torque_step_nm": max_torque_step_nm,
        "min_contact_activations": min_contact_activations,
        "max_source_age_ms": max_source_age_ms,
        "max_record_gap_ms": max_record_gap_ms,
    }
    for key, value in limits.items():
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) <= 0.0
        ):
            raise RuntimeError(f"{key} must be finite and positive")
    if int(min_contact_activations) != min_contact_activations:
        raise RuntimeError("min_contact_activations must be an integer")

    target = Path(path).expanduser().resolve()
    if not target.is_file() or target.stat().st_size <= 0:
        raise RuntimeError(f"evidence CSV is missing or empty: {target}")
    with target.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        missing = sorted(ONSET_REQUIRED_COLUMNS.difference(reader.fieldnames or ()))
        if missing:
            raise RuntimeError(f"{target}: CSV columns are missing: {missing}")
        rows = [row for row in reader if row.get("state", "").strip().upper() == "FAST"]
    if len(rows) < 2:
        raise RuntimeError(f"{target}: no usable FAST interval")

    times = np.asarray([_number(row, "t_s") for row in rows])
    if np.any(np.diff(times) <= 0.0):
        raise RuntimeError(f"{target}: FAST timestamps are not increasing")
    record_gaps_ms = np.diff(times) * 1000.0
    valid_values = np.asarray([_number(row, "observer_valid") for row in rows])
    ready_values = np.asarray([_number(row, "observer_model_ready") for row in rows])
    contact_values = np.asarray(
        [_number(row, "observer_contact_state") for row in rows]
    )
    for name, values in (
        ("observer_valid", valid_values),
        ("observer_model_ready", ready_values),
        ("observer_contact_state", contact_values),
    ):
        if np.any(~np.isin(values, (0.0, 1.0))):
            raise RuntimeError(f"{target}: {name} must contain only 0 or 1")
    source_age_ms = np.asarray(
        [_number(row, "observer_source_age_ms") for row in rows]
    )
    local_age_ms = np.asarray([_number(row, "fe_age_ms") for row in rows])
    scale = np.asarray([_number(row, "contact_scale") for row in rows])
    gain_scale = np.asarray(
        [_number(row, "feedback_gain_scale_contract") for row in rows]
    )
    feedback = np.asarray(
        [[_number(row, f"tau_fb_j{index}") for index in range(1, 7)] for row in rows]
    )
    allowed = (
        (valid_values == 1.0)
        & (ready_values == 1.0)
        & (source_age_ms >= -2.0)
        & (source_age_ms <= float(max_source_age_ms))
        & (local_age_ms >= 0.0)
        & (local_age_ms <= float(max_source_age_ms))
        & (contact_values == 1.0)
    )
    starts = np.flatnonzero(allowed & ~np.r_[False, allowed[:-1]])
    failures = []
    if len(starts) and starts[0] == 0:
        failures.append("first CONTACT starts before the FAST evidence window")

    episodes = []
    for start in starts[starts > 0]:
        inactive = np.flatnonzero(~allowed[start + 1:])
        end = start + 1 + int(inactive[0]) if len(inactive) else len(rows)
        full = np.flatnonzero(scale[start:end] >= 1.0 - 1.0e-4)
        episode = {
            "onset_time_s": float(times[start]),
            "initial_scale": float(scale[start]),
            "completed": bool(len(full)),
            "feedback_peak_nm": float(np.max(np.abs(feedback[start:end]))),
        }
        if scale[start] > 1.0e-4:
            failures.append(f"CONTACT at row {start + 2}: ramp did not start at zero")
        if episode["feedback_peak_nm"] <= 1.0e-6:
            failures.append(f"CONTACT at row {start + 2}: feedback stayed zero")
        if not len(full):
            failures.append(f"CONTACT at row {start + 2}: ramp did not reach scale 1")
            episode.update({"rise_time_ms": None, "max_torque_step_nm": None})
            episodes.append(episode)
            continue
        full_index = start + int(full[0])
        rise_time_ms = float((times[full_index] - times[start]) * 1000.0)
        torque_steps = np.abs(np.diff(feedback[start - 1:full_index + 1], axis=0))
        max_step_nm = float(np.max(torque_steps))
        episode.update(
            {
                "rise_time_ms": rise_time_ms,
                "max_torque_step_nm": max_step_nm,
            }
        )
        episodes.append(episode)
        if np.any(np.diff(scale[start:full_index + 1]) < -1.0e-4):
            failures.append(f"CONTACT at row {start + 2}: ramp scale moved backwards")
        if rise_time_ms > float(max_rise_time_ms) + 1.0e-9:
            failures.append(f"CONTACT at row {start + 2}: rise time exceeded the limit")
        if max_step_nm > float(max_torque_step_nm) + 1.0e-9:
            failures.append(f"CONTACT at row {start + 2}: torque step exceeded the limit")

    if len(episodes) < int(min_contact_activations):
        failures.append("evaluable CONTACT activations are insufficient")
    if np.max(record_gaps_ms) > float(max_record_gap_ms) + 1.0e-9:
        failures.append("FAST record gap exceeded the limit")
    if np.min(gain_scale) <= 0.0 or np.max(gain_scale) - np.min(gain_scale) > 1.0e-9:
        failures.append("evidence must use one nonzero feedback gain stage")
    if np.any((scale < -1.0e-4) | (scale > 1.0 + 1.0e-4)):
        failures.append("contact scale is outside [0, 1]")
    blocked = ~allowed
    blocked_feedback_max = float(np.max(np.abs(feedback[blocked]))) if np.any(blocked) else 0.0
    blocked_scale_max = float(np.max(np.abs(scale[blocked]))) if np.any(blocked) else 0.0
    if blocked_feedback_max > 1.0e-6:
        failures.append("feedback torque is nonzero while canonical feedback is blocked")
    if blocked_scale_max > 1.0e-6:
        failures.append("contact scale is nonzero while canonical feedback is blocked")

    completed = [episode for episode in episodes if episode["completed"]]
    return {
        "schema_version": ANALYSIS_SCHEMA_VERSION,
        "analysis_type": ONSET_ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "source": {
            "path": str(target),
            "sha256": file_sha256(target),
            "size_bytes": target.stat().st_size,
        },
        "limits": limits,
        "aggregate": {
            "evaluable_contact_activations": len(episodes),
            "completed_ramps": len(completed),
            "feedback_gain_scale": float(np.min(gain_scale)),
            "max_rise_time_ms": max(
                (episode["rise_time_ms"] for episode in completed), default=None
            ),
            "max_torque_step_nm": max(
                (episode["max_torque_step_nm"] for episode in completed), default=None
            ),
            "blocked_feedback_abs_max_nm": blocked_feedback_max,
            "blocked_scale_abs_max": blocked_scale_max,
        },
        "episodes": episodes,
        "failures": failures,
    }


def analyze_feedback_evidence(
    model_path,
    target_gain_scale,
    free_csvs,
    contact_csvs,
    limits,
):
    validate_limits(limits)
    target_gain_scale = float(target_gain_scale)
    if target_gain_scale not in TARGET_TO_EVIDENCE_GAIN:
        raise RuntimeError("target gain must be exactly 0.40 or 1.00")
    evidence_gain = TARGET_TO_EVIDENCE_GAIN[target_gain_scale]
    model = Path(model_path).expanduser().resolve()
    if model.is_dir():
        model = model / "model.ts"
    metadata = model.parent / "metadata.json"
    if not model.is_file() or not metadata.is_file():
        raise RuntimeError("accepted model.ts and metadata.json are required")
    model_document = json.loads(metadata.read_text(encoding="utf-8"))
    runtime_model_acceptance(
        model_document, file_sha256(model), file_sha256(metadata)
    )

    evidence_paths = [
        Path(path).expanduser().resolve() for path in (*free_csvs, *contact_csvs)
    ]
    if len(set(evidence_paths)) != len(evidence_paths):
        raise RuntimeError("each FREE/CONTACT evidence CSV must be a distinct file")
    free_runs = [analyze_csv(path, "free") for path in free_csvs]
    contact_runs = [analyze_csv(path, "contact") for path in contact_csvs]
    all_runs = free_runs + contact_runs
    failures = []

    def fail(condition, message):
        if condition:
            failures.append(message)

    fail(len(free_runs) < 3, "at least three independent FREE runs are required")
    fail(not contact_runs, "at least one controlled CONTACT run is required")
    for run in all_runs:
        label = Path(run["path"]).name
        fail(
            run["duration_s"] < limits["min_run_duration_s"],
            f"{label}: FAST duration is below the minimum",
        )
        fail(
            run["actual_hz"] < limits["min_csv_hz"],
            f"{label}: Leader CSV rate is below the minimum",
        )
        fail(
            abs(run["feedback_gain_scale_min"] - evidence_gain) > 1.0e-9
            or abs(run["feedback_gain_scale_max"] - evidence_gain) > 1.0e-9,
            f"{label}: logged feedback gain stage does not match evidence stage",
        )
        fail(run["invalid_samples"] != 0, f"{label}: observer invalid samples exist")
        fail(
            run["model_not_ready_samples"] != 0,
            f"{label}: model-not-ready samples exist",
        )
        fail(
            run["source_age_min_ms"] < -2.0
            or run["source_age_max_ms"] > limits["max_source_age_ms"],
            f"{label}: observer source age is outside the contract",
        )
        fail(
            run["prediction_age_min_ms"] < 0.0
            or run["observer_latency_min_ms"] < 0.0
            or run["prediction_age_max_ms"] > limits["max_source_age_ms"]
            or run["observer_latency_max_ms"] > limits["max_source_age_ms"],
            f"{label}: observer prediction age or latency exceeded the contract",
        )
        fail(
            run["max_dt_ms"] > limits["max_record_gap_ms"],
            f"{label}: CSV record gap exceeded the limit",
        )
        fail(
            run["score_force_norm_difference_max_n"] > 0.01,
            f"{label}: contact score and logged force norm disagree",
        )
        fail(
            max(run["leader_pose_step_max_deg"], run["follower_pose_step_max_deg"])
            > limits["max_pose_step_deg"],
            f"{label}: pose jump exceeded the limit",
        )
        fail(
            run["leader_velocity_reversal_hz_max"]
            > limits["max_velocity_reversal_hz"],
            f"{label}: velocity reversal rate exceeded the vibration limit",
        )

    false_contacts = sum(run["contact_activations"] for run in free_runs)
    free_force_p95 = max(
        (run["force_norm_p95_n"] for run in free_runs), default=math.inf
    )
    free_force_p99 = max(
        (run["force_norm_p99_n"] for run in free_runs), default=math.inf
    )
    free_force_max = max((run["force_norm_max_n"] for run in free_runs), default=math.inf)
    contact_force_max = max(
        (run["force_norm_max_n"] for run in contact_runs), default=math.inf
    )
    contact_activations = sum(run["contact_activations"] for run in contact_runs)
    free_feedback_max = max(
        (run["feedback_abs_max_nm"] for run in free_runs), default=math.inf
    )
    feedback_max_by_joint = np.max(
        np.asarray([run["feedback_abs_max_nm_by_joint"] for run in all_runs]), axis=0
    )
    contact_rows = sum(
        run["fast_rows"] * run["contact_fraction"] for run in contact_runs
    )
    contact_nonzero = sum(
        run["fast_rows"]
        * run["contact_fraction"]
        * run["contact_feedback_nonzero_fraction"]
        for run in contact_runs
    )
    contact_nonzero_fraction = contact_nonzero / contact_rows if contact_rows else 0.0

    fail(false_contacts != 0, "FREE runs contain false CONTACT activations")
    fail(
        free_force_p95 > limits["max_free_force_p95_n"],
        "FREE residual force p95 exceeded the operational limit",
    )
    fail(
        free_force_p99 > limits["max_free_force_p99_n"],
        "FREE residual force p99 exceeded the operational limit",
    )
    fail(
        free_force_max > limits["max_free_force_error_n"],
        "FREE residual force exceeded the operational hard limit",
    )
    fail(free_feedback_max > 1.0e-6, "feedback torque leaked into canonical FREE")
    fail(
        contact_activations < limits["min_contact_activations"],
        "controlled CONTACT activations are insufficient",
    )
    fail(
        contact_force_max > limits["max_contact_force_n"],
        "controlled CONTACT force exceeded the operator limit",
    )
    stage_clip = np.asarray(REFERENCE_CLIP_NM) * evidence_gain
    if evidence_gain == 0.0:
        fail(float(np.max(feedback_max_by_joint)) > 1.0e-6, "feedback-OFF torque is nonzero")
    else:
        fail(
            bool(np.any(feedback_max_by_joint > stage_clip + 1.1e-4)),
            "40-percent feedback exceeded its absolute torque clip",
        )
        fail(
            contact_nonzero_fraction
            < limits["min_contact_feedback_nonzero_fraction"],
            "CONTACT feedback nonzero fraction is insufficient",
        )

    return {
        "schema_version": ANALYSIS_SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "target_gain_scale": target_gain_scale,
        "evidence_gain_scale": evidence_gain,
        "model_binding": {
            "model_path": str(model),
            "model_sha256": file_sha256(model),
            "metadata_path": str(metadata),
            "metadata_sha256": file_sha256(metadata),
        },
        "limits": limits,
        "aggregate": {
            "free_run_count": len(free_runs),
            "contact_run_count": len(contact_runs),
            "false_contact_activations": false_contacts,
            "controlled_contact_activations": contact_activations,
            "free_force_norm_p95_n": free_force_p95,
            "free_force_norm_p99_n": free_force_p99,
            "free_force_norm_max_n": free_force_max,
            "contact_force_norm_max_n": contact_force_max,
            "free_feedback_abs_max_nm": free_feedback_max,
            "feedback_abs_max_nm_by_joint": feedback_max_by_joint.tolist(),
            "contact_feedback_nonzero_fraction": contact_nonzero_fraction,
            "pose_step_max_deg": max(
                max(run["leader_pose_step_max_deg"], run["follower_pose_step_max_deg"])
                for run in all_runs
            ),
            "velocity_reversal_hz_max": max(
                run["leader_velocity_reversal_hz_max"] for run in all_runs
            ),
        },
        "runs": all_runs,
        "failures": failures,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Analyze FT feedback CSV evidence for the next authorized stage"
    )
    parser.add_argument("--model", required=True)
    parser.add_argument("--target-gain-scale", required=True, type=float, choices=(0.40, 1.00))
    parser.add_argument("--free-csv", required=True, nargs="+")
    parser.add_argument("--contact-csv", required=True, nargs="+")
    parser.add_argument("--max-contact-force-n", required=True, type=float)
    parser.add_argument(
        "--max-free-force-p95-n",
        type=float,
        default=OPERATIONAL_FREE_FORCE_P95_LIMIT_N,
    )
    parser.add_argument(
        "--max-free-force-p99-n",
        type=float,
        default=OPERATIONAL_FREE_FORCE_P99_LIMIT_N,
    )
    parser.add_argument(
        "--max-free-force-error-n",
        type=float,
        default=OPERATIONAL_FREE_FORCE_HARD_MAX_LIMIT_N,
    )
    parser.add_argument("--max-pose-step-deg", type=float, default=1.0)
    parser.add_argument("--max-velocity-reversal-hz", type=float, default=8.0)
    parser.add_argument("--max-source-age-ms", type=float, default=20.0)
    parser.add_argument("--max-record-gap-ms", type=float, default=10.0)
    parser.add_argument("--min-run-duration-s", type=float, default=10.0)
    parser.add_argument("--min-csv-hz", type=float, default=250.0)
    parser.add_argument("--min-contact-activations", type=int, default=3)
    parser.add_argument("--min-contact-feedback-nonzero-fraction", type=float, default=0.95)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    try:
        args = parse_args(argv)
        numeric_limits = {
            "max_contact_force_n": args.max_contact_force_n,
            "max_free_force_p95_n": args.max_free_force_p95_n,
            "max_free_force_p99_n": args.max_free_force_p99_n,
            "max_free_force_error_n": args.max_free_force_error_n,
            "max_pose_step_deg": args.max_pose_step_deg,
            "max_velocity_reversal_hz": args.max_velocity_reversal_hz,
            "max_source_age_ms": args.max_source_age_ms,
            "max_record_gap_ms": args.max_record_gap_ms,
            "min_run_duration_s": args.min_run_duration_s,
            "min_csv_hz": args.min_csv_hz,
            "min_contact_activations": args.min_contact_activations,
            "min_contact_feedback_nonzero_fraction": (
                args.min_contact_feedback_nonzero_fraction
            ),
        }
        validate_limits(numeric_limits)
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite analysis report: {output}")
        report = analyze_feedback_evidence(
            args.model,
            args.target_gain_scale,
            args.free_csv,
            args.contact_csv,
            numeric_limits,
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(("GO" if report["passed"] else "NO-GO") + f": {output}")
        for failure in report["failures"]:
            print(f"- {failure}")
        return 0 if report["passed"] else 2
    except Exception as exc:
        print(f"ERROR: FT feedback analysis failed: {exc}", file=sys.stderr)
        return 1


def onset_main(argv=None):
    parser = argparse.ArgumentParser(
        description="Evaluate canonical CONTACT feedback ramp-in from a leader CSV"
    )
    parser.add_argument("--csv", required=True)
    parser.add_argument("--max-rise-time-ms", required=True, type=float)
    parser.add_argument("--max-torque-step-nm", required=True, type=float)
    parser.add_argument("--min-contact-activations", type=int, default=3)
    parser.add_argument("--max-source-age-ms", type=float, default=20.0)
    parser.add_argument("--max-record-gap-ms", type=float, default=10.0)
    parser.add_argument("--output", required=True)
    try:
        args = parser.parse_args(argv)
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite onset report: {output}")
        report = analyze_feedback_onsets(
            args.csv,
            args.max_rise_time_ms,
            args.max_torque_step_nm,
            args.min_contact_activations,
            args.max_source_age_ms,
            args.max_record_gap_ms,
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
        print(f"ERROR: feedback onset analysis failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
