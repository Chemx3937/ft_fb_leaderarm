#!/usr/bin/env python3
"""Compare stored V7 and current free-space runtime residuals on identical FREE masks."""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import tempfile

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from analyze_logistic_box_il_replay import (
    guarded_free_mask,
    metric_from_error,
    read_array,
    read_json,
)
from ft_fb_leaderarm.model import file_sha256


DEFAULT_DATA = Path("/data/logistic_box_contact_observer")
DEFAULT_CURRENT_ANALYSIS = (
    PACKAGE_ROOT
    / "document/experiment/free_space_wrench_model_validation/analysis.json"
)
DEFAULT_LEGACY_MODEL = Path(
    "/home/vision/dualarm_ws/src/fb_leaderarm/data/free_space_wrench_models/"
    "right_v7_task_domain_observation_epoch8_20260725/model.ts"
)
DEFAULT_OUTPUT = (
    PACKAGE_ROOT / "document/experiment/free_space_wrench_model_comparison"
)
EXPECTED_CURRENT_MODEL_SHA256 = (
    "8c61261bb2fdd0151291f9c52ca627e59e04d71bc5655c92df5081943280ee8b"
)
EXPECTED_CURRENT_METADATA_SHA256 = (
    "025d761ba285d34850dfe4da1ba9b89d6f7c2109f9a03181fdfbadb55463d882"
)
EXPECTED_LEGACY_MODEL_SHA256 = (
    "74bc5c4d16d7f167ac8d490116d74d94ce36a09fe3b22010fa943029c36018c2"
)
EXPECTED_LEGACY_METADATA_SHA256 = (
    "8e807831bcf35d6cf33ca3bef7c8ef4216a30f68db3e2eac3e52dbae4ca01e96"
)
EXPECTED_EPISODES = 102
FREE_GUARD_MS = (0, 50, 100, 200, 500)
PRIMARY_FREE_GUARD_MS = 200
FORCE_METRICS = (
    "force_norm_mean_n",
    "force_norm_rmse_n",
    "force_norm_p95_n",
    "force_norm_p99_n",
    "force_norm_max_n",
    "force_within_1n_fraction",
    "force_within_2n_fraction",
)
LOWER_IS_BETTER = (
    "force_norm_mean_n",
    "force_norm_rmse_n",
    "force_norm_p95_n",
    "force_norm_p99_n",
    "force_norm_max_n",
)
PLOT_METRICS = (
    ("force_norm_rmse_n", "RMSE [N]"),
    ("force_norm_p95_n", "p95 [N]"),
    ("force_norm_p99_n", "p99 [N]"),
    ("force_norm_max_n", "maximum [N]"),
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument(
        "--current-analysis", type=Path, default=DEFAULT_CURRENT_ANALYSIS
    )
    parser.add_argument("--legacy-model", type=Path, default=DEFAULT_LEGACY_MODEL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args(argv)


def require_equal(actual, expected, label):
    if actual != expected:
        raise RuntimeError(f"{label} mismatch: expected {expected!r}, got {actual!r}")


def require_hash(path, expected, label):
    path = Path(path)
    if not path.is_file():
        raise RuntimeError(f"missing {label}: {path}")
    actual = file_sha256(path)
    require_equal(actual, expected, f"{label} SHA-256")
    return actual


def force_metrics(wrench):
    wrench = np.asarray(wrench, dtype=np.float64)
    if wrench.ndim != 2 or wrench.shape[1] != 6 or len(wrench) < 2:
        raise RuntimeError("force metrics require at least two 6-axis wrench samples")
    if not np.isfinite(wrench).all():
        raise RuntimeError("wrench samples must be finite")
    metrics = metric_from_error(wrench)
    return {"samples": int(metrics["samples"]), **{key: metrics[key] for key in FORCE_METRICS}}


def force_metrics_from_report(metrics):
    missing = ({"samples", *FORCE_METRICS}) - set(metrics)
    if missing:
        raise RuntimeError(f"current report is missing metrics: {sorted(missing)}")
    return {
        "samples": int(metrics["samples"]),
        **{key: float(metrics[key]) for key in FORCE_METRICS},
    }


def metric_deltas(current, legacy):
    require_equal(current["samples"], legacy["samples"], "metric sample count")
    absolute = {}
    relative_pct = {}
    for key in FORCE_METRICS:
        absolute[key] = float(current[key] - legacy[key])
        relative_pct[key] = (
            float(100.0 * absolute[key] / legacy[key])
            if legacy[key] != 0.0
            else None
        )
    return {"absolute": absolute, "relative_percent": relative_pct}


def validate_artifacts(data_root, current_analysis_path, legacy_model_path):
    if not data_root.is_dir():
        raise RuntimeError(f"missing dataset: {data_root}")
    if not current_analysis_path.is_file():
        raise RuntimeError(f"missing current analysis: {current_analysis_path}")

    current_report = read_json(current_analysis_path)
    require_equal(current_report.get("schema_version"), 1, "current report schema")
    require_equal(
        current_report.get("analysis_type"),
        "logistic_box_il_current_free_space_model_replay_v1",
        "current analysis type",
    )
    require_equal(current_report.get("episodes"), EXPECTED_EPISODES, "episode count")
    require_equal(
        Path(current_report.get("dataset", "")).resolve(),
        data_root,
        "current report dataset",
    )
    require_equal(
        current_report.get("model", {}).get("sha256"),
        EXPECTED_CURRENT_MODEL_SHA256,
        "current report model SHA-256",
    )

    current_model = Path(current_report["model"]["path"]).expanduser().resolve()
    current_metadata = current_model.with_name("metadata.json")
    legacy_metadata = legacy_model_path.with_name("metadata.json")
    hashes = {
        "current_model": require_hash(
            current_model, EXPECTED_CURRENT_MODEL_SHA256, "current model"
        ),
        "current_metadata": require_hash(
            current_metadata, EXPECTED_CURRENT_METADATA_SHA256, "current metadata"
        ),
        "legacy_model": require_hash(
            legacy_model_path, EXPECTED_LEGACY_MODEL_SHA256, "legacy model"
        ),
        "legacy_metadata": require_hash(
            legacy_metadata, EXPECTED_LEGACY_METADATA_SHA256, "legacy metadata"
        ),
    }
    legacy_metadata_json = read_json(legacy_metadata)
    require_equal(legacy_metadata_json.get("schema_version"), 3, "legacy metadata schema")
    require_equal(
        legacy_metadata_json.get("model_type"),
        "free_space_wrench_lstm_summary_ensemble_v3",
        "legacy model type",
    )
    runtime = legacy_metadata_json.get("detector_aligned_evaluation_contract", {})
    require_equal(runtime.get("prediction_lpf_enabled"), True, "legacy prediction LPF")
    require_equal(runtime.get("prediction_lpf_cutoff_hz"), 80.0, "legacy LPF cutoff")
    return current_report, current_model, current_metadata, legacy_metadata, hashes


def audit_observer_logs(data_root, legacy_model_path):
    rows = []
    paths = sorted((data_root / "observer_logs").glob("*/*.npz"))
    if not paths:
        raise RuntimeError("no observer logs found")
    for path in paths:
        with np.load(path, allow_pickle=False) as data:
            if "metadata" not in data.files:
                rows.append({"path": str(path), "status": "metadata_missing"})
                continue
            metadata = json.loads(str(data["metadata"].item()))
        declared_path = Path(metadata.get("model_path", "")).expanduser().resolve()
        declared_sha = (
            metadata.get("model_metadata", {})
            .get("artifact_integrity", {})
            .get("binary_sha256", {})
            .get("model.ts")
        )
        diagnostics = metadata.get("observer_diagnostics", {})
        require_equal(declared_path, legacy_model_path, f"observer log model path {path}")
        require_equal(
            declared_sha,
            EXPECTED_LEGACY_MODEL_SHA256,
            f"observer log model SHA-256 {path}",
        )
        require_equal(
            diagnostics.get("residual_bias_calibration_enabled"),
            True,
            f"observer log residual bias {path}",
        )
        rows.append(
            {
                "path": str(path),
                "status": "verified",
                "model_sha256": declared_sha,
                "residual_bias_calibration_enabled": True,
                "residual_bias_calibration_ms": diagnostics.get(
                    "residual_bias_calibration_ms"
                ),
            }
        )
    return {
        "logs_checked": len(rows),
        "verified": sum(row["status"] == "verified" for row in rows),
        "metadata_missing": sum(row["status"] == "metadata_missing" for row in rows),
        "details": rows,
    }


def episode_masks(episode, current_row, history):
    meta = read_json(episode / "meta.json")
    require_equal(
        file_sha256(episode / "meta.json"),
        current_row["episode_meta_sha256"],
        f"{episode.name} metadata SHA-256",
    )
    source_t = read_array(episode, "contact/source_time_stamps.zarr")
    receive_t = read_array(episode, "contact/receive_time_stamps.zarr")
    sequence = read_array(episode, "contact/source_sequences.zarr").astype(np.int64)
    reference = read_array(episode, "contact/contact_state.zarr").astype(bool)
    valid = read_array(episode, "contact/contact_valid.zarr").astype(bool)
    ready = read_array(episode, "contact/contact_model_ready.zarr").astype(bool)
    legacy_wrench = read_array(episode, "contact/contact_wrench.zarr")
    values = (receive_t, sequence, reference, valid, ready, legacy_wrench)
    if not all(len(value) == len(source_t) for value in values):
        raise RuntimeError(f"contact array length mismatch: {episode}")
    if legacy_wrench.shape != (len(source_t), 6):
        raise RuntimeError(f"invalid legacy contact wrench: {episode}")

    same_identity = (np.diff(source_t) == 0.0) & (np.diff(sequence) == 0)
    if np.any((np.diff(source_t) <= 0.0) & ~same_identity) or np.any(
        (np.diff(sequence) <= 0) & ~same_identity
    ):
        raise RuntimeError(f"contact source identity moved backwards: {episode}")
    keep = np.r_[True, ~same_identity]
    source_t, receive_t, reference, valid, ready, legacy_wrench = (
        value[keep]
        for value in (source_t, receive_t, reference, valid, ready, legacy_wrench)
    )
    if np.any(np.diff(source_t) <= 0.0) or np.any(np.diff(receive_t) <= 0.0):
        raise RuntimeError(f"contact timestamps do not increase: {episode}")

    official = (source_t >= float(meta["created_at"])) & (
        source_t <= float(meta["stopped_at"])
    )
    warm = np.arange(len(source_t)) >= history - 1
    usable = official & valid & ready & warm
    require_equal(int(np.sum(~keep)), current_row["deduplicated_samples"], f"{episode.name} dedupe count")
    require_equal(int(np.sum(usable)), current_row["usable_samples"], f"{episode.name} usable count")
    masks = {
        str(guard): usable & guarded_free_mask(reference, source_t, guard)
        for guard in FREE_GUARD_MS
    }
    for guard, mask in masks.items():
        require_equal(
            int(np.sum(mask)),
            current_row["free_samples_by_guard_ms"][guard],
            f"{episode.name} {guard} ms FREE count",
        )
    return legacy_wrench, masks


def count_wins(episode_rows):
    result = {}
    for key in LOWER_IS_BETTER:
        legacy = np.asarray([row["legacy"][key] for row in episode_rows])
        current = np.asarray([row["current"][key] for row in episode_rows])
        tied = np.isclose(current, legacy, rtol=1e-12, atol=1e-12)
        result[key] = {
            "current_lower": int(np.sum((current < legacy) & ~tied)),
            "legacy_lower": int(np.sum((legacy < current) & ~tied)),
            "tied": int(np.sum(tied)),
        }
    return result


def analyze_episodes(data_root, current_report):
    episodes = sorted(data_root.glob("episode_[0-9][0-9][0-9]"))
    expected_names = [f"episode_{index:03d}" for index in range(EXPECTED_EPISODES)]
    require_equal([path.name for path in episodes], expected_names, "contiguous episodes")
    current_rows = current_report.get("episodes_detail", [])
    require_equal(len(current_rows), EXPECTED_EPISODES, "current episode detail count")
    current_by_name = {row["episode"]: row for row in current_rows}
    require_equal(sorted(current_by_name), expected_names, "current episode identities")
    history = int(current_report["model"]["history"])
    legacy_by_guard = {str(guard): [] for guard in FREE_GUARD_MS}
    episode_rows = []

    for episode in episodes:
        current_row = current_by_name[episode.name]
        legacy_wrench, masks = episode_masks(episode, current_row, history)
        legacy_metrics = {
            guard: force_metrics(legacy_wrench[mask]) for guard, mask in masks.items()
        }
        for guard, mask in masks.items():
            legacy_by_guard[guard].append(legacy_wrench[mask])
        current_metrics = {
            guard: force_metrics_from_report(
                current_row["free_space_metrics_by_guard_ms"][guard]
            )
            for guard in map(str, FREE_GUARD_MS)
        }
        for guard in map(str, FREE_GUARD_MS):
            require_equal(
                legacy_metrics[guard]["samples"],
                current_metrics[guard]["samples"],
                f"{episode.name} {guard} ms paired samples",
            )
        primary = str(PRIMARY_FREE_GUARD_MS)
        episode_rows.append(
            {
                "episode": episode.name,
                "input_fidelity": current_row["input_fidelity"],
                "samples": legacy_metrics[primary]["samples"],
                "legacy": legacy_metrics[primary],
                "current": current_metrics[primary],
                "delta": metric_deltas(
                    current_metrics[primary], legacy_metrics[primary]
                ),
            }
        )

    guard_sweep = {}
    for guard in map(str, FREE_GUARD_MS):
        legacy = force_metrics(np.concatenate(legacy_by_guard[guard]))
        current = force_metrics_from_report(
            current_report["aggregate_by_contact_guard_ms"][guard]
        )
        guard_sweep[guard] = {
            "samples": legacy["samples"],
            "legacy": legacy,
            "current": current,
            "delta": metric_deltas(current, legacy),
        }
    return episode_rows, guard_sweep


def write_csv(path, rows):
    fieldnames = ["episode", "input_fidelity", "samples"]
    for key in FORCE_METRICS:
        fieldnames.extend(
            (f"legacy_{key}", f"current_{key}", f"delta_{key}", f"delta_percent_{key}")
        )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            flat = {key: row[key] for key in fieldnames[:3]}
            for key in FORCE_METRICS:
                flat[f"legacy_{key}"] = row["legacy"][key]
                flat[f"current_{key}"] = row["current"][key]
                flat[f"delta_{key}"] = row["delta"]["absolute"][key]
                flat[f"delta_percent_{key}"] = row["delta"]["relative_percent"][key]
            writer.writerow(flat)


def plot_summary(path, report):
    rows = report["episodes_detail"]
    primary = report["primary"]
    fig, axes = plt.subplots(2, 3, figsize=(15, 9), constrained_layout=True)
    labels = [label.split()[0] for _, label in PLOT_METRICS]
    x = np.arange(len(labels))
    width = 0.36
    axes[0, 0].bar(
        x - width / 2,
        [primary["legacy"][key] for key, _ in PLOT_METRICS],
        width,
        label="legacy V7 runtime",
    )
    axes[0, 0].bar(
        x + width / 2,
        [primary["current"][key] for key, _ in PLOT_METRICS],
        width,
        label="current runtime",
    )
    axes[0, 0].set_xticks(x, labels)
    axes[0, 0].set_ylabel("force residual [N]")
    axes[0, 0].set_title("Aggregate stable FREE (200 ms)")
    axes[0, 0].legend()

    guards = list(map(str, FREE_GUARD_MS))
    axes[0, 1].plot(
        FREE_GUARD_MS,
        [report["guard_sweep"][guard]["legacy"]["force_norm_p99_n"] for guard in guards],
        marker="o",
        label="legacy V7",
    )
    axes[0, 1].plot(
        FREE_GUARD_MS,
        [report["guard_sweep"][guard]["current"]["force_norm_p99_n"] for guard in guards],
        marker="o",
        label="current",
    )
    axes[0, 1].set_xlabel("contact guard [ms]")
    axes[0, 1].set_ylabel("p99 [N]")
    axes[0, 1].set_title("Guard sensitivity")
    axes[0, 1].legend()

    scatter_axes = (axes[0, 2], axes[1, 0], axes[1, 1], axes[1, 2])
    for axis, (key, label) in zip(scatter_axes, PLOT_METRICS):
        legacy = np.asarray([row["legacy"][key] for row in rows])
        current = np.asarray([row["current"][key] for row in rows])
        limit = float(max(np.max(legacy), np.max(current)) * 1.05)
        axis.scatter(legacy, current, s=16, alpha=0.7)
        axis.plot([0.0, limit], [0.0, limit], "k--", linewidth=1)
        axis.set_xlim(0.0, limit)
        axis.set_ylim(0.0, limit)
        axis.set_xlabel("legacy V7")
        axis.set_ylabel("current")
        wins = report["episode_win_counts"][key]["current_lower"]
        axis.set_title(f"{label}: current lower {wins}/{len(rows)}")
    fig.suptitle("Free-space runtime residual comparison (diagnostic only)")
    fig.savefig(path, dpi=180)
    plt.close(fig)


def report_readme(report):
    primary = report["primary"]
    old = primary["legacy"]
    new = primary["current"]
    delta = primary["delta"]["relative_percent"]
    audit = report["observer_log_contract_audit"]
    return f"""# Free-space wrench 실사용 residual 비교

## 결론

동일한 stable-FREE 200 ms 구간 {primary['samples']:,}샘플에서 current pipeline은
V7 pipeline보다 일반 오차가 작았다. RMSE는 {old['force_norm_rmse_n']:.3f} N에서
{new['force_norm_rmse_n']:.3f} N({delta['force_norm_rmse_n']:+.1f}%), p95는
{old['force_norm_p95_n']:.3f} N에서 {new['force_norm_p95_n']:.3f} N
({delta['force_norm_p95_n']:+.1f}%), p99는 {old['force_norm_p99_n']:.3f} N에서
{new['force_norm_p99_n']:.3f} N({delta['force_norm_p99_n']:+.1f}%)으로 줄었다.

반면 hard max는 {old['force_norm_max_n']:.3f} N에서
{new['force_norm_max_n']:.3f} N({delta['force_norm_max_n']:+.1f}%)으로 커졌다.
따라서 결론은 **일반 오차 개선, 극단 오차 악화**이며 단일 승자를 선언하지 않는다.

| force norm | Legacy V7 | Current | 변화율 |
|---|---:|---:|---:|
| mean | {old['force_norm_mean_n']:.3f} N | {new['force_norm_mean_n']:.3f} N | {delta['force_norm_mean_n']:+.1f}% |
| RMSE | {old['force_norm_rmse_n']:.3f} N | {new['force_norm_rmse_n']:.3f} N | {delta['force_norm_rmse_n']:+.1f}% |
| p95 | {old['force_norm_p95_n']:.3f} N | {new['force_norm_p95_n']:.3f} N | {delta['force_norm_p95_n']:+.1f}% |
| p99 | {old['force_norm_p99_n']:.3f} N | {new['force_norm_p99_n']:.3f} N | {delta['force_norm_p99_n']:+.1f}% |
| hard max | {old['force_norm_max_n']:.3f} N | {new['force_norm_max_n']:.3f} N | {delta['force_norm_max_n']:+.1f}% |
| within 1 N | {old['force_within_1n_fraction']:.1%} | {new['force_within_1n_fraction']:.1%} | {delta['force_within_1n_fraction']:+.1f}% |
| within 2 N | {old['force_within_2n_fraction']:.1%} | {new['force_within_2n_fraction']:.1%} | {delta['force_within_2n_fraction']:+.1f}% |

## 육하원칙과 비교 계약

- 누가/언제: offline 비교기가 {report['created_utc']}에 저장 artifact를 읽었다.
- 어디서: `{report['dataset']['path']}`의 연속 102개 episode를 사용했다.
- 무엇을: 저장된 V7 `contact_wrench`와 [current replay 결과](../free_space_wrench_model_validation/README.md)의 force residual norm을 비교했다.
- 어떻게: valid/model-ready/current 32-sample warmup 조건과 legacy contact state 기준 0/50/100/200/500 ms guard를 동일하게 적용했다. primary는 200 ms다.
- 왜: 모델 구조 자체가 아니라 실제 배치된 두 pipeline의 FREE residual trade-off를 진단하기 위해서다.
- 모델 증거: observer log {audit['logs_checked']}개 중 {audit['verified']}개에서 V7 SHA와 startup residual bias 활성화를 확인했고, metadata가 없는 {audit['metadata_missing']}개는 확인 불가로 기록했다.

V7은 `/bae_r/F_e(filtered)`를 `right_base_link`에서 예측하고 80 Hz prediction LPF와
startup residual bias를 사용한다. Current 모델은 `/aft_sensor2/wrench`를 sensor frame에서
예측한다. 회전에 불변인 force norm만 비교하며 moment와 축별 값은 비교하지 않는다.

## 산출물

- [집계 및 계약 JSON](analysis.json)
- [episode별 비교 CSV](episode_metrics.csv)
- [요약 그림](summary.png)

## 제한과 다음 실험

- FREE mask가 V7의 저장 contact state에서 만들어져 V7에 유리한 선택 편향이 있다.
- 56개 episode의 current 입력은 30 Hz joint interpolation이다.
- 서로 다른 target/frame의 실사용 residual 비교이므로 FS-03 또는 CO-04 evidence가 아니다.
- 최종 모델 교체 판단 전 별도 승인 아래 독립 zero-set 3개 이상에서 두 target을 동시에 저장하고, 동일 시간 구간의 RMSE/p95/p99/max와 FREE false CONTACT를 다시 비교한다.

## 재현

```bash
python3 scripts/compare_free_space_wrench_runtime_residuals.py --self-check
python3 scripts/compare_free_space_wrench_runtime_residuals.py \
  --output /tmp/free_space_wrench_model_comparison_recheck
```
"""


def analyze(args):
    data_root = args.data.expanduser().resolve()
    current_analysis_path = args.current_analysis.expanduser().resolve()
    legacy_model_path = args.legacy_model.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {output}")

    current_report, current_model, current_metadata, legacy_metadata, hashes = (
        validate_artifacts(data_root, current_analysis_path, legacy_model_path)
    )
    log_audit = audit_observer_logs(data_root, legacy_model_path)
    episode_rows, guard_sweep = analyze_episodes(data_root, current_report)
    primary = guard_sweep[str(PRIMARY_FREE_GUARD_MS)]
    report = {
        "schema_version": 1,
        "analysis_type": "free_space_wrench_runtime_residual_comparison_v1",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "script_sha256": file_sha256(Path(__file__)),
        "artifacts": {
            "legacy": {
                "model_path": str(legacy_model_path),
                "model_sha256": hashes["legacy_model"],
                "metadata_path": str(legacy_metadata),
                "metadata_sha256": hashes["legacy_metadata"],
                "target": "ObserverInput.measured_wrench:/bae_r/F_e(filtered)",
                "frame": "right_base_link",
                "pipeline": "stored V7 residual after 80 Hz prediction LPF and startup residual bias",
            },
            "current": {
                "model_path": str(current_model),
                "model_sha256": hashes["current_model"],
                "metadata_path": str(current_metadata),
                "metadata_sha256": hashes["current_metadata"],
                "source_analysis": str(current_analysis_path),
                "source_analysis_sha256": file_sha256(current_analysis_path),
                "target": "ft/wrench_raw.zarr from /aft_sensor2/wrench",
                "frame": "aft_sensor2 sensor frame",
                "pipeline": "physical payload gravity plus learned residual",
            },
        },
        "dataset": {
            "path": str(data_root),
            "episodes": len(episode_rows),
            "primary_samples": primary["samples"],
        },
        "evaluation_contract": {
            "comparison_scope": "deployed runtime residual pipelines, not isolated predictors",
            "metric": "Euclidean force residual norm in N",
            "primary_contact_guard_ms": PRIMARY_FREE_GUARD_MS,
            "contact_guard_sweep_ms": list(FREE_GUARD_MS),
            "shared_mask": "official interval AND stored valid/model_ready AND current 32-sample warmup AND guarded legacy FREE",
            "moment_or_axis_comparison": False,
            "formal_fs03_or_co04_evidence": False,
        },
        "observer_log_contract_audit": log_audit,
        "primary": primary,
        "guard_sweep": guard_sweep,
        "episode_win_counts": count_wins(episode_rows),
        "episodes_detail": episode_rows,
        "conclusion": "typical error improved; hard maximum regressed; no single winner",
        "limitations": [
            "the legacy observer contact state defines FREE and can selection-bias the comparison toward V7",
            "the models predict different sensor targets and frames",
            "56 current-model episodes use 30 Hz joint interpolation",
            "one observer log lacks embedded metadata and cannot independently prove its V7 contract",
            "no independent same-clock contact ground truth is available",
        ],
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".wrench_compare_", dir=output.parent) as temp:
        staging = Path(temp) / output.name
        staging.mkdir()
        (staging / "analysis.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        write_csv(staging / "episode_metrics.csv", episode_rows)
        plot_summary(staging / "summary.png", report)
        (staging / "README.md").write_text(report_readme(report), encoding="utf-8")
        staging.rename(output)
    print(
        f"[done] paired_samples={primary['samples']}, "
        f"legacy_p99={primary['legacy']['force_norm_p99_n']:.3f} N, "
        f"current_p99={primary['current']['force_norm_p99_n']:.3f} N, output={output}",
        flush=True,
    )


def self_check():
    wrench = np.zeros((4, 6), dtype=np.float64)
    wrench[:, 0] = (0.5, 1.0, 2.0, 2.5)
    legacy = force_metrics(wrench)
    current = force_metrics(wrench * 0.5)
    delta = metric_deltas(current, legacy)
    assert legacy["force_within_1n_fraction"] == 0.5
    assert current["force_norm_rmse_n"] < legacy["force_norm_rmse_n"]
    assert delta["relative_percent"]["force_norm_rmse_n"] == -50.0
    reference = np.asarray([False, False, True, True, False, False])
    times = np.arange(6, dtype=np.float64) * 0.01
    assert guarded_free_mask(reference, times, 10).tolist() == [
        True,
        False,
        False,
        False,
        False,
        True,
    ]
    for actual, expected, label in (("bad", "good", "hash"), (3, 4, "samples")):
        try:
            require_equal(actual, expected, label)
        except RuntimeError:
            pass
        else:
            raise AssertionError(f"{label} mismatch was not rejected")
    print("free-space runtime residual comparison self-check: PASS")


def main(argv=None):
    args = parse_args(argv)
    if args.self_check:
        self_check()
        return 0
    analyze(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
