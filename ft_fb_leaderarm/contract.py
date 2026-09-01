"""Shared data, feature, frame, and detector contracts."""

from collections import deque
import math

import numpy as np


SCHEMA_VERSION = 1
APPROVAL_CONTRACT = "robust_force_v2_262p5hz"
OPERATOR_SELECTED_MODEL_CONTRACT = "operator_selected_20260901"
OPERATOR_SELECTED_MODEL_SHA256 = (
    "8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b"
)
OPERATOR_SELECTED_METADATA_SHA256 = (
    "025d761ba285d34850dfe4da1ba9b89d6f7c2109f9a03181fdfbadb55463d882"
)
SAMPLE_HZ = 262.5
SAMPLE_PERIOD_S = 1.0 / SAMPLE_HZ
BASE_FEATURE_DIM = 18
WRENCH_DIM = 6
FORCE_P99_LIMIT_N = 1.0
FORCE_GROUP_P95_LIMIT_N = 1.0
FORCE_HARD_MAX_LIMIT_N = 2.0
OPERATIONAL_FREE_FORCE_P95_LIMIT_N = 1.2
OPERATIONAL_FREE_FORCE_P99_LIMIT_N = 1.5
OPERATIONAL_FREE_FORCE_HARD_MAX_LIMIT_N = 2.5
INFERENCE_P99_LIMIT_MS = 0.80 * SAMPLE_PERIOD_S * 1000.0
INFERENCE_HARD_MAX_LIMIT_MS = SAMPLE_PERIOD_S * 1000.0
DEFAULT_ZERO_POSE_DEG = (5.5, 52.0, 112.0, 28.0, -107.0, -35.0)
ABLATIONS = {
    "static_linear": ("static", 1, (), "mlp"),
    "dynamic_mlp": ("dynamic", 1, (128, 128), "mlp"),
    "history_mlp": ("history", 16, (128, 128), "mlp"),
    "history_lstm": ("sequence", 16, (128,), "lstm"),
    "history_gru": ("sequence", 16, (128,), "gru"),
}


def runtime_model_acceptance(metadata, model_sha256, metadata_sha256):
    """Return the exact contract that permits a bundle to run."""
    if metadata.get("model_sha256") != model_sha256:
        raise RuntimeError("model SHA-256 does not match metadata")
    if metadata.get("approved") is True:
        if metadata.get("approval_contract") != APPROVAL_CONTRACT:
            raise RuntimeError("model approval contract is missing or obsolete")
        return APPROVAL_CONTRACT
    if (
        model_sha256 == OPERATOR_SELECTED_MODEL_SHA256
        and metadata_sha256 == OPERATOR_SELECTED_METADATA_SHA256
    ):
        return OPERATOR_SELECTED_MODEL_CONTRACT
    raise RuntimeError("model is rejected; it is not approved or operator-selected")


def finite_vector(value, size):
    array = np.asarray(value, dtype=np.float64).reshape(-1)
    if array.shape != (size,) or not np.isfinite(array).all():
        return None
    return array


def stamp_to_seconds(stamp):
    if stamp is None:
        return 0.0
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def wrench_from_message(msg):
    return np.asarray(
        [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ],
        dtype=np.float64,
    )


def fill_wrench_message(msg, values):
    values = finite_vector(values, WRENCH_DIM)
    if values is None:
        raise ValueError("wrench must contain six finite values")
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = values[:3]
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = values[3:]
    return msg


class RateGate:
    """Select at most one source sample for each configured time slot."""

    def __init__(self, sample_hz=SAMPLE_HZ):
        if not math.isfinite(sample_hz) or sample_hz <= 0.0:
            raise ValueError("sample_hz must be positive")
        self.period_s = 1.0 / float(sample_hz)
        self.next_stamp_s = None

    def reset(self):
        self.next_stamp_s = None

    def accept(self, stamp_s):
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return False
        if self.next_stamp_s is None:
            self.next_stamp_s = stamp_s + self.period_s
            return True
        if stamp_s + 1.0e-12 < self.next_stamp_s:
            return False
        skipped = max(1, int((stamp_s - self.next_stamp_s) / self.period_s) + 1)
        self.next_stamp_s += skipped * self.period_s
        return True


class CausalFeatureBuilder:
    """Build [q,dq,qdd] without using future samples."""

    def __init__(self, max_dt_s=0.05):
        self.max_dt_s = float(max_dt_s)
        self.previous_dq = None
        self.previous_stamp_s = None

    def reset(self):
        self.previous_dq = None
        self.previous_stamp_s = None

    def build(self, q_rad, dq_rad_s, stamp_s):
        q = finite_vector(q_rad, 6)
        dq = finite_vector(dq_rad_s, 6)
        stamp_s = float(stamp_s)
        if q is None or dq is None or not math.isfinite(stamp_s) or stamp_s <= 0.0:
            return None
        qdd = np.zeros(6, dtype=np.float64)
        if self.previous_dq is not None and self.previous_stamp_s is not None:
            dt_s = stamp_s - self.previous_stamp_s
            if 0.0 < dt_s <= self.max_dt_s:
                qdd = (dq - self.previous_dq) / dt_s
        self.previous_dq = dq.copy()
        self.previous_stamp_s = stamp_s
        feature = np.concatenate((q, dq, qdd))
        return feature if np.isfinite(feature).all() else None


class FixedPoseZeroVerifier:
    """Require a stable, near-zero FT stream at the configured initial pose."""

    def __init__(
        self,
        zero_pose_deg=DEFAULT_ZERO_POSE_DEG,
        pose_tolerance_deg=1.0,
        max_joint_speed_rad_s=0.02,
        settle_s=1.0,
        force_norm_max_n=1.0,
        force_axis_std_max_n=0.40,
        minimum_samples=100,
    ):
        pose = finite_vector(zero_pose_deg, 6)
        if pose is None:
            raise ValueError("zero_pose_deg must contain six finite values")
        self.zero_pose_rad = np.deg2rad(pose)
        self.pose_tolerance_rad = math.radians(float(pose_tolerance_deg))
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self.settle_s = float(settle_s)
        self.force_norm_max_n = float(force_norm_max_n)
        self.force_axis_std_max_n = float(force_axis_std_max_n)
        self.minimum_samples = int(minimum_samples)
        self.samples = deque()
        self.last_reason = "waiting_for_samples"

    def reset(self, reason="reset"):
        self.samples.clear()
        self.last_reason = str(reason)

    def update(self, monotonic_s, q_rad, dq_rad_s, wrench):
        now_s = float(monotonic_s)
        q = finite_vector(q_rad, 6)
        dq = finite_vector(dq_rad_s, 6)
        wrench = finite_vector(wrench, 6)
        if q is None or dq is None or wrench is None or not math.isfinite(now_s):
            self.reset("nonfinite_input")
            return False
        if float(np.max(np.abs(q - self.zero_pose_rad))) > self.pose_tolerance_rad:
            self.reset("not_at_fixed_zero_pose")
            return False
        if float(np.max(np.abs(dq))) > self.max_joint_speed_rad_s:
            self.reset("robot_not_stationary")
            return False
        self.samples.append((now_s, wrench.copy()))
        cutoff = now_s - self.settle_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()
        ready, reason = self.status()
        self.last_reason = reason
        return ready

    def status(self):
        if len(self.samples) < self.minimum_samples:
            return False, "insufficient_zero_samples"
        duration_s = self.samples[-1][0] - self.samples[0][0]
        if duration_s < 0.95 * self.settle_s:
            return False, "zero_settle_incomplete"
        force = np.stack([row[1][:3] for row in self.samples], axis=0)
        center_norm = float(np.linalg.norm(np.median(force, axis=0)))
        if center_norm > self.force_norm_max_n:
            return False, "zero_force_offset_too_large"
        if float(np.max(np.std(force, axis=0))) > self.force_axis_std_max_n:
            return False, "zero_force_noise_too_large"
        return True, "zero_verified"

    def summary(self):
        ready, reason = self.status()
        result = {"ready": ready, "reason": reason, "samples": len(self.samples)}
        if self.samples:
            force = np.stack([row[1][:3] for row in self.samples], axis=0)
            result.update(
                {
                    "duration_s": self.samples[-1][0] - self.samples[0][0],
                    "force_median_n": np.median(force, axis=0).tolist(),
                    "force_std_n": np.std(force, axis=0).tolist(),
                }
            )
        return result


def project_feature_windows(windows, mode):
    """Project [N,H,18] causal source windows for one ablation."""
    array = np.asarray(windows, dtype=np.float32)
    if array.ndim != 3 or array.shape[2] != BASE_FEATURE_DIM:
        raise ValueError("feature windows must have shape [N,H,18]")
    q = array[:, :, :6]
    transformed = np.concatenate(
        (np.sin(q), np.cos(q), array[:, :, 6:12], array[:, :, 12:18]), axis=2
    )
    if mode == "static":
        return transformed[:, -1, :12]
    if mode == "dynamic":
        return transformed[:, -1, :]
    if mode == "history":
        return transformed.reshape(len(transformed), -1)
    if mode == "sequence":
        return transformed
    if mode == "short_multiscale":
        if array.shape[1] < 32:
            raise ValueError("short_multiscale requires at least 32 samples")
        q = array[:, -1, :6]
        parts = [np.sin(q), np.cos(q), array[:, -1, 6:12]]
        parts.extend(
            np.mean(array[:, -width:, 6:12], axis=1)
            for width in (8, 16, 32)
        )
        parts.extend(
            np.mean(array[:, -width:, 12:18], axis=1)
            for width in (8, 16, 32)
        )
        return np.concatenate(parts, axis=1)
    raise ValueError(f"unsupported feature mode: {mode}")


def rotation_zyx_deg(angles_deg):
    angles = finite_vector(angles_deg, 3)
    if angles is None:
        raise ValueError("Euler angles must contain three finite values")
    z, y, x = np.deg2rad(angles)
    cz, sz = math.cos(z), math.sin(z)
    cy, sy = math.cos(y), math.sin(y)
    cx, sx = math.cos(x), math.sin(x)
    rz = np.asarray(((cz, -sz, 0.0), (sz, cz, 0.0), (0.0, 0.0, 1.0)))
    ry = np.asarray(((cy, 0.0, sy), (0.0, 1.0, 0.0), (-sy, 0.0, cy)))
    rx = np.asarray(((1.0, 0.0, 0.0), (0.0, cx, -sx), (0.0, sx, cx)))
    return rz @ ry @ rx


def sensor_wrench_to_base(
    wrench_sensor,
    current_pose,
    sensor_to_tip_rpy_deg=(0.0, 0.0, 0.0),
    tip_to_sensor_translation_m=(0.0, 0.0, 0.0),
):
    """Transform [F,M] at the sensor into base axes at the follower tip."""
    wrench = finite_vector(wrench_sensor, 6)
    pose = finite_vector(current_pose, 6)
    offset = finite_vector(tip_to_sensor_translation_m, 3)
    if wrench is None or pose is None or offset is None:
        raise ValueError("wrench, current_pose, and sensor offset must be finite")
    r_base_tip = rotation_zyx_deg(pose[3:])
    r_tip_sensor = rotation_zyx_deg(sensor_to_tip_rpy_deg)
    force_tip = r_tip_sensor @ wrench[:3]
    moment_tip = r_tip_sensor @ wrench[3:] + np.cross(offset, force_tip)
    return np.concatenate((r_base_tip @ force_tip, r_base_tip @ moment_tip))


class SchmittContactDetector:
    def __init__(self, force_on_n, force_off_n, contact_hold_s, free_hold_s):
        if force_on_n <= force_off_n or force_off_n < 0.0:
            raise ValueError("force thresholds must satisfy on > off >= 0")
        self.force_on_n = float(force_on_n)
        self.force_off_n = float(force_off_n)
        self.contact_hold_s = float(contact_hold_s)
        self.free_hold_s = float(free_hold_s)
        self.reset()

    def reset(self):
        self.contact = False
        self.pending_since_s = None

    def update(self, force_norm_n, stamp_s):
        value = float(force_norm_n)
        stamp_s = float(stamp_s)
        threshold_met = value <= self.force_off_n if self.contact else value >= self.force_on_n
        if not threshold_met:
            self.pending_since_s = None
            return self.contact
        if self.pending_since_s is None:
            self.pending_since_s = stamp_s
        hold_s = self.free_hold_s if self.contact else self.contact_hold_s
        if stamp_s - self.pending_since_s >= hold_s:
            self.contact = not self.contact
            self.pending_since_s = None
        return self.contact


def error_metrics(target, prediction):
    target = np.asarray(target, dtype=np.float64)
    prediction = np.asarray(prediction, dtype=np.float64)
    if target.shape != prediction.shape or target.ndim != 2 or target.shape[1] != 6:
        raise ValueError("target and prediction must both have shape [N,6]")
    error = target - prediction
    force_norm = np.linalg.norm(error[:, :3], axis=1)
    return {
        "samples": int(len(error)),
        "force_norm_max_n": float(np.max(force_norm)),
        "force_norm_p95_n": float(np.percentile(force_norm, 95.0)),
        "force_norm_p99_n": float(np.percentile(force_norm, 99.0)),
        "force_norm_rmse_n": float(np.sqrt(np.mean(np.square(force_norm)))),
        "force_axis_abs_max_n": np.max(np.abs(error[:, :3]), axis=0).tolist(),
        "moment_axis_abs_max_nm": np.max(np.abs(error[:, 3:]), axis=0).tolist(),
        "wrench_axis_rmse": np.sqrt(np.mean(np.square(error), axis=0)).tolist(),
    }


def robust_force_accuracy_gate(
    overall,
    by_group,
    p99_limit_n=FORCE_P99_LIMIT_N,
    group_p95_limit_n=FORCE_GROUP_P95_LIMIT_N,
    hard_max_limit_n=FORCE_HARD_MAX_LIMIT_N,
):
    if not by_group:
        raise ValueError("at least one zero-set group is required")
    limits = [p99_limit_n, group_p95_limit_n, hard_max_limit_n]
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("force accuracy limits must be finite and positive")
    if hard_max_limit_n < max(p99_limit_n, group_p95_limit_n):
        raise ValueError("hard max limit cannot be below a percentile limit")
    p99 = float(overall["force_norm_p99_n"])
    hard_max = float(overall["force_norm_max_n"])
    group_p95 = {
        str(name): float(metrics["force_norm_p95_n"])
        for name, metrics in by_group.items()
    }
    values = [p99, hard_max, *group_p95.values()]
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError("force accuracy metrics must be finite and non-negative")
    failures = []
    if p99 > p99_limit_n:
        failures.append("aggregate force p99 exceeds limit")
    if hard_max > hard_max_limit_n:
        failures.append("aggregate force hard max exceeds limit")
    failed_groups = sorted(
        name for name, value in group_p95.items() if value > group_p95_limit_n
    )
    if failed_groups:
        failures.append("zero-set group force p95 exceeds limit")
    return {
        "passed": not failures,
        "limits": {
            "force_norm_p99_n": float(p99_limit_n),
            "force_group_p95_n": float(group_p95_limit_n),
            "force_hard_max_n": float(hard_max_limit_n),
        },
        "metrics": {
            "force_norm_p99_n": p99,
            "force_norm_max_n": hard_max,
            "force_group_p95_n": group_p95,
            "failed_groups": failed_groups,
        },
        "failures": failures,
    }


def episode_timing_quality(
    stamps_s,
    sample_hz=SAMPLE_HZ,
    minimum_duration_s=10.0,
    max_record_gap_s=0.010,
    invalid_samples=0,
):
    stamps = np.asarray(stamps_s, dtype=np.float64).reshape(-1)
    count = len(stamps)
    ordered = (
        count >= 2
        and np.isfinite(stamps).all()
        and bool(np.all(np.diff(stamps) > 0.0))
    )
    duration_s = float(stamps[-1] - stamps[0]) if ordered else 0.0
    actual_hz = (count - 1) / duration_s if duration_s > 0.0 else 0.0
    max_gap_s = float(np.max(np.diff(stamps))) if ordered else math.inf
    accepted = (
        ordered
        and duration_s >= float(minimum_duration_s)
        and count >= int(float(minimum_duration_s) * float(sample_hz))
        and abs(actual_hz - float(sample_hz)) <= 0.03 * float(sample_hz)
        and max_gap_s <= float(max_record_gap_s)
        and int(invalid_samples) == 0
    )
    return {
        "accepted": bool(accepted),
        "samples": count,
        "duration_s": duration_s,
        "actual_hz": actual_hz,
        "max_record_gap_s": max_gap_s,
    }
