"""Fail-closed, artifact-bound gain stages for physical-FT feedback."""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

import yaml

STAGE_SCALES = (0.40, 1.00)
REFERENCE_GAIN = (0.012, 0.012, 0.065, 0.012, 0.065, 0.012)
REFERENCE_CLIP_NM = (0.30, 0.30, 0.30, 0.08, 0.30, 0.08)
REFERENCE_SLEW_NM_S = (1.5, 1.5, 2.5, 0.6, 2.5, 0.6)
MAX_STALE_TIMEOUT_S = 0.020
AUTHORIZATION_SCHEMA_VERSION = 1
AUTHORIZATION_TYPE = "physical_ft_feedback_stage_v1"
ATTESTATIONS = {
    1: (
        "I verified at least three feedback-OFF free-space runs, controlled "
        "contact detection, and the passing automatic analysis"
    ),
    2: (
        "I verified correct feedback direction with no vibration or pose jump "
        "at the 40 percent stage and reviewed the passing automatic analysis"
    ),
}
ANALYSIS_SCHEMA_VERSION = 1
ANALYSIS_TYPE = "physical_ft_feedback_analysis_v1"
TARGET_TO_EVIDENCE_GAIN = {0.40: 0.0, 1.00: 0.40}


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stage_for_scale(gain_scale):
    value = float(gain_scale)
    if not math.isfinite(value):
        raise RuntimeError("feedback_gain_scale must be finite")
    for index, allowed in enumerate(STAGE_SCALES, start=1):
        if abs(value - allowed) <= 1.0e-9:
            return index
    raise RuntimeError(
        "feedback_gain_scale must be exactly one of 0.40 or 1.00"
    )


def _approved_model_contract(model_path):
    model = Path(model_path).expanduser().resolve()
    if model.is_dir():
        model = model / "model.ts"
    metadata = model.parent / "metadata.json"
    if not model.is_file() or not metadata.is_file():
        raise RuntimeError("model.ts and metadata.json are both required")
    try:
        document = json.loads(metadata.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"failed to read model metadata: {metadata}") from exc
    if document.get("approved") is not True:
        raise RuntimeError("feedback authorization requires approved=true")
    model_hash = file_sha256(model)
    if document.get("model_sha256") != model_hash:
        raise RuntimeError("model SHA-256 differs from metadata")
    return model, metadata, model_hash, file_sha256(metadata)


def _validate_analysis_report(path, model, metadata, target_gain_scale):
    target = Path(path).expanduser().resolve()
    try:
        document = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"failed to read FT feedback analysis: {target}") from exc
    expected_evidence_gain = TARGET_TO_EVIDENCE_GAIN[float(target_gain_scale)]
    if (
        document.get("schema_version") != ANALYSIS_SCHEMA_VERSION
        or document.get("analysis_type") != ANALYSIS_TYPE
        or document.get("passed") is not True
        or document.get("failures") != []
        or document.get("target_gain_scale") != float(target_gain_scale)
        or document.get("evidence_gain_scale") != expected_evidence_gain
    ):
        raise RuntimeError(
            "feedback authorization requires a passing analysis for the exact stage"
        )
    binding = document.get("model_binding")
    if not isinstance(binding, dict):
        raise RuntimeError("feedback analysis model binding is missing")
    if (
        Path(str(binding.get("model_path", ""))).expanduser().resolve() != model
        or Path(str(binding.get("metadata_path", ""))).expanduser().resolve()
        != metadata
        or binding.get("model_sha256") != file_sha256(model)
        or binding.get("metadata_sha256") != file_sha256(metadata)
    ):
        raise RuntimeError("feedback analysis model binding changed")
    runs = document.get("runs")
    if not isinstance(runs, list) or not runs:
        raise RuntimeError("feedback analysis contains no raw CSV bindings")
    for run in runs:
        if not isinstance(run, dict):
            raise RuntimeError("feedback analysis run binding is malformed")
        raw = Path(str(run.get("path", ""))).expanduser().resolve()
        if (
            not raw.is_file()
            or run.get("size_bytes") != raw.stat().st_size
            or run.get("sha256") != file_sha256(raw)
        ):
            raise RuntimeError(f"feedback analysis raw CSV changed: {raw}")
    from .feedback_analysis import analyze_feedback_evidence

    free_csvs = [run["path"] for run in runs if run.get("expected_state") == "free"]
    contact_csvs = [
        run["path"] for run in runs if run.get("expected_state") == "contact"
    ]
    recomputed = analyze_feedback_evidence(
        model,
        target_gain_scale,
        free_csvs,
        contact_csvs,
        document.get("limits"),
    )
    for key in (
        "schema_version",
        "analysis_type",
        "passed",
        "target_gain_scale",
        "evidence_gain_scale",
        "model_binding",
        "limits",
        "aggregate",
        "runs",
        "failures",
    ):
        if document.get(key) != recomputed.get(key):
            raise RuntimeError(
                f"feedback analysis {key} does not match recomputed raw CSV metrics"
            )
    return document


def _evidence_binding(paths, model, metadata, target_gain_scale):
    if len(paths) != 1:
        raise RuntimeError("exactly one passing FT feedback analysis JSON is required")
    result = []
    for value in paths:
        path = Path(value).expanduser().resolve()
        if not path.is_file() or path.stat().st_size <= 0:
            raise RuntimeError(f"evidence file is missing or empty: {path}")
        _validate_analysis_report(path, model, metadata, target_gain_scale)
        result.append(
            {
                "path": str(path),
                "size_bytes": path.stat().st_size,
                "sha256": file_sha256(path),
            }
        )
    if not result:
        raise RuntimeError("a passing FT feedback analysis is required")
    return result


def validate_feedback_authorization(
    authorization_path, model_path, gain_scale, _seen=None
):
    stage = stage_for_scale(gain_scale)
    model, metadata, model_hash, metadata_hash = _approved_model_contract(model_path)
    target = Path(authorization_path).expanduser().resolve()
    if not target.is_file():
        raise RuntimeError(f"feedback authorization is missing: {target}")
    seen = set() if _seen is None else _seen
    if target in seen:
        raise RuntimeError("feedback authorization chain contains a cycle")
    seen.add(target)
    try:
        document = json.loads(target.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"failed to read feedback authorization: {target}") from exc
    expected = {
        "schema_version": AUTHORIZATION_SCHEMA_VERSION,
        "authorization_type": AUTHORIZATION_TYPE,
        "authorized": True,
        "stage": stage,
        "gain_scale": STAGE_SCALES[stage - 1],
        "operator_attestation": ATTESTATIONS[stage],
        "model_path": str(model),
        "model_metadata_path": str(metadata),
        "model_sha256": model_hash,
        "model_metadata_sha256": metadata_hash,
    }
    for key, value in expected.items():
        if document.get(key) != value:
            raise RuntimeError(f"feedback authorization {key} does not match")
    evidence = document.get("evidence")
    if not isinstance(evidence, list) or len(evidence) != 1:
        raise RuntimeError("feedback authorization requires exactly one analysis")
    for item in evidence:
        if not isinstance(item, dict):
            raise RuntimeError("feedback authorization evidence is malformed")
        path = Path(str(item.get("path", ""))).expanduser().resolve()
        if (
            not path.is_file()
            or item.get("size_bytes") != path.stat().st_size
            or item.get("sha256") != file_sha256(path)
        ):
            raise RuntimeError(f"feedback evidence binding changed: {path}")
        _validate_analysis_report(path, model, metadata, gain_scale)
    previous_value = str(document.get("previous_authorization", "")).strip()
    if stage == 1:
        if previous_value:
            raise RuntimeError("stage-1 authorization cannot bind a previous stage")
    else:
        if not previous_value:
            raise RuntimeError("higher feedback stage requires previous authorization")
        previous = Path(previous_value).expanduser().resolve()
        if document.get("previous_authorization_sha256") != file_sha256(previous):
            raise RuntimeError("previous feedback authorization hash changed")
        validate_feedback_authorization(
            previous, model, STAGE_SCALES[stage - 2], seen
        )
    return document


def create_feedback_authorization(
    output_path,
    model_path,
    gain_scale,
    evidence_paths,
    operator_attestation,
    previous_authorization="",
):
    stage = stage_for_scale(gain_scale)
    if str(operator_attestation) != ATTESTATIONS[stage]:
        raise RuntimeError(
            "operator attestation must exactly match the required stage text"
        )
    model, metadata, model_hash, metadata_hash = _approved_model_contract(model_path)
    previous = str(previous_authorization).strip()
    if stage == 1 and previous:
        raise RuntimeError("stage-1 authorization cannot have a previous stage")
    if stage > 1:
        if not previous:
            raise RuntimeError("higher feedback stage requires previous authorization")
        previous_path = Path(previous).expanduser().resolve()
        validate_feedback_authorization(
            previous_path, model, STAGE_SCALES[stage - 2]
        )
        previous = str(previous_path)
    document = {
        "schema_version": AUTHORIZATION_SCHEMA_VERSION,
        "authorization_type": AUTHORIZATION_TYPE,
        "authorized": True,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "stage": stage,
        "gain_scale": STAGE_SCALES[stage - 1],
        "operator_attestation": ATTESTATIONS[stage],
        "model_path": str(model),
        "model_metadata_path": str(metadata),
        "model_sha256": model_hash,
        "model_metadata_sha256": metadata_hash,
        "evidence": _evidence_binding(
            evidence_paths, model, metadata, gain_scale
        ),
        "previous_authorization": previous,
        "previous_authorization_sha256": (
            file_sha256(previous) if previous else ""
        ),
    }
    output = Path(output_path).expanduser().resolve()
    if output.exists() or output.is_symlink():
        raise RuntimeError(f"refusing to overwrite feedback authorization: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True), encoding="utf-8")
    validate_feedback_authorization(output, model, gain_scale)
    return output


def _vector6(parameters, name, fallback=None):
    value = parameters.get(name, fallback)
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"{name} must be a numeric six-vector") from exc
    if len(result) != 6 or any(not math.isfinite(item) or item < 0.0 for item in result):
        raise RuntimeError(f"{name} must contain six finite non-negative values")
    return result


def scaled_leader_overrides(leader_config, side, gain_scale):
    try:
        document = yaml.safe_load(Path(leader_config).read_text(encoding="utf-8"))
        parameters = document["leader_teleop_node"]["ros__parameters"]
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise RuntimeError(f"invalid leader config: {leader_config}") from exc
    gain = _vector6(
        parameters,
        f"{side}_jt_wrench_fb_gain",
        parameters.get("jt_wrench_fb_gain", [0.0] * 6),
    )
    if gain_scale == 0.0:
        return {f"{side}_jt_wrench_fb_gain": [0.0] * 6}
    clip = _vector6(parameters, "jt_wrench_fb_clip")
    slew = _vector6(parameters, "tau_fb_slew_rate_Nm_s")

    def capped(values, reference):
        return [
            min(gain_scale * value, gain_scale * limit)
            for value, limit in zip(values, reference)
        ]

    return {
        f"{side}_jt_wrench_fb_gain": capped(gain, REFERENCE_GAIN),
        "jt_wrench_fb_clip": capped(clip, REFERENCE_CLIP_NM),
        "tau_fb_slew_rate_Nm_s": capped(slew, REFERENCE_SLEW_NM_S),
        "tau_fb_motion_gate_enable": True,
        "tau_fb_motion_gate_speed_source": "joint_max",
        "tau_fb_motion_gate_speed_low_rad_s": 0.05,
        "tau_fb_motion_gate_speed_high_rad_s": 0.20,
        "tau_fb_motion_gate_min_scale": 0.05,
        "tau_fb_passivity_gate_enable": True,
        "tau_fb_passivity_power_start_W": 0.001,
        "tau_fb_passivity_power_full_W": 0.012,
        "tau_fb_passivity_min_scale": 0.05,
        "csv_log_enabled": True,
    }
