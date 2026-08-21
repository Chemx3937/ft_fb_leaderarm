#!/usr/bin/env python3
"""Screen train13-only free-space wrench improvements on validation3."""

import argparse
import hashlib
import json
import math
import subprocess
import sys
import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as functional
from torch.utils.data import DataLoader, TensorDataset, WeightedRandomSampler

import ft_fb_leaderarm.train_ablation as training
from ft_fb_leaderarm.contract import error_metrics
from ft_fb_leaderarm.model import WrenchRegressor, make_normalized_model


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import run_physical_residual_validation as physical  # noqa: E402
import run_targeted6_validation_ablation as targeted  # noqa: E402
import run_causal_residual_filter_validation as filtering  # noqa: E402


DEFAULT_OUTPUT = Path(
    "/home/vision/.ros/ft_fb_leaderarm/models/"
    "right_train13_improvement_sweep_v2_20260822"
)
COMMON_HISTORY = 128
SEED = 8
SCREEN_EPOCHS = 60
MAX_WINDOWS = 20_000

CONFIGS = (
    ("baseline_recheck", "base", (128, 128), "baseline", "random"),
    ("zero_median_feature", "zero", (128, 128), "baseline", "random"),
    ("session_weighted_sampling", "base", (128, 128), "baseline", "session"),
    ("motion_weighted_sampling", "base", (128, 128), "baseline", "motion"),
    ("huber_loss", "base", (128, 128), "huber", "random"),
    ("mae_mse_loss", "base", (128, 128), "mae_mse", "random"),
    ("force_tail_loss", "base", (128, 128), "force_tail", "random"),
    ("small_mlp", "base", (64, 64), "baseline", "random"),
    ("large_mlp", "base", (256, 256, 128), "baseline", "random"),
    ("elapsed_feature", "elapsed", (128, 128), "baseline", "random"),
    ("multiscale_history", "multiscale", (128, 128), "baseline", "random"),
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def zero_force(session):
    return np.asarray(
        session.metadata["zero_verification"]["force_median_n"],
        dtype=np.float32,
    )


def rolling_mean(values, ends, width):
    cumulative = np.vstack(
        (np.zeros((1, values.shape[1]), dtype=np.float64), np.cumsum(values, axis=0))
    )
    return (cumulative[ends + 1] - cumulative[ends + 1 - width]) / width


def feature_history(kind):
    if kind == "short_multiscale":
        return 32
    return COMMON_HISTORY if kind == "multiscale" else 8


def project_features(session, ends, kind):
    q = session.features[ends, :6]
    dq = session.features[ends, 6:12]
    qdd8 = rolling_mean(session.features[:, 12:18], ends, 8)
    base = np.column_stack((np.sin(q), np.cos(q), dq, qdd8))
    if kind == "base":
        return base
    if kind == "zero":
        return np.column_stack((base, np.broadcast_to(zero_force(session), (len(ends), 3))))
    if kind == "elapsed":
        elapsed = (ends / float(session.metadata["sample_hz"]))[:, None]
        return np.column_stack((base, elapsed))
    if kind == "short_multiscale":
        widths = (8, 16, 32)
        dq_means = [
            rolling_mean(session.features[:, 6:12], ends, width)
            for width in widths
        ]
        qdd_means = [
            rolling_mean(session.features[:, 12:18], ends, width)
            for width in widths
        ]
        return np.column_stack((np.sin(q), np.cos(q), dq, *dq_means, *qdd_means))
    if kind != "multiscale":
        raise ValueError(f"unknown feature kind: {kind}")
    dq_means = [rolling_mean(session.features[:, 6:12], ends, width) for width in (8, 32, 128)]
    qdd_means = [rolling_mean(session.features[:, 12:18], ends, width) for width in (8, 32, 128)]
    q_deltas = [q - session.features[ends + 1 - width, :6] for width in (8, 32, 128)]
    return np.column_stack(
        (
            np.sin(q),
            np.cos(q),
            dq,
            *dq_means,
            *qdd_means,
            *q_deltas,
            np.sign(dq),
        )
    )


def build_data(
    sessions, kind, seed, validation=False, common_history=COMMON_HISTORY
):
    history = feature_history(kind)
    x_parts, y_parts, group_parts, acceleration_parts = [], [], [], []
    for session_index, session in enumerate(sessions):
        first = max(history, common_history if validation else history) - 1
        ends = np.arange(first, len(session.features))
        if not validation and len(ends) > MAX_WINDOWS:
            rng = np.random.default_rng(seed + session_index)
            ends = np.sort(rng.choice(ends, MAX_WINDOWS, replace=False))
        x_parts.append(project_features(session, ends, kind))
        y_parts.append(session.wrench[ends])
        group_parts.append(np.full(len(ends), session_index, dtype=np.int16))
        acceleration_parts.append(np.linalg.norm(session.features[ends, 12:18], axis=1))
    return {
        "x": np.concatenate(x_parts).astype(np.float32),
        "y": np.concatenate(y_parts).astype(np.float32),
        "group": np.concatenate(group_parts),
        "acceleration": np.concatenate(acceleration_parts).astype(np.float32),
        "group_names": [session.group for session in sessions],
    }


def sampler_for(data, kind, seed):
    if kind == "random":
        return None
    labels = data["group"] if kind == "session" else np.digitize(
        data["acceleration"], (1.0, 3.0, 6.0)
    )
    counts = np.bincount(labels)
    weights = 1.0 / counts[labels]
    return WeightedRandomSampler(
        torch.as_tensor(weights, dtype=torch.double),
        num_samples=len(weights),
        replacement=True,
        generator=torch.Generator().manual_seed(seed),
    )


def loss_value(error, kind):
    force_norm_sq = torch.sum(error[:, :3] ** 2, dim=1)
    tail_count = max(1, int(math.ceil(0.05 * len(force_norm_sq))))
    tail = torch.mean(torch.topk(force_norm_sq, tail_count).values)
    if kind == "baseline":
        return torch.mean(error**2) + 0.20 * tail
    if kind == "huber":
        return functional.smooth_l1_loss(
            error, torch.zeros_like(error), beta=0.5
        ) + 0.10 * tail
    if kind == "mae_mse":
        return 0.50 * torch.mean(error**2) + 0.50 * torch.mean(torch.abs(error)) + 0.10 * tail
    if kind == "force_tail":
        return torch.mean(error[:, :3] ** 2) + 0.10 * torch.mean(error[:, 3:] ** 2) + 0.50 * tail
    raise ValueError(f"unknown loss: {kind}")


def metrics_from_prediction(data, prediction):
    by_group = {}
    for index, name in enumerate(data["group_names"]):
        mask = data["group"] == index
        by_group[name] = error_metrics(data["y"][mask], prediction[mask])
    return error_metrics(data["y"], prediction), by_group


def subset_data(data, mask):
    return {
        "x": data["x"][mask],
        "y": data["y"][mask],
        "group": data["group"][mask],
        "acceleration": data["acceleration"][mask],
        "group_names": data["group_names"],
    }


def train_mlp(train, validation, hidden, loss_kind, sampling, seed=SEED):
    x_mean = train["x"].mean(axis=0)
    x_std = np.maximum(train["x"].std(axis=0), 1.0e-6)
    y_mean = train["y"].mean(axis=0)
    y_std = np.maximum(train["y"].std(axis=0), 1.0e-6)
    train_x = (train["x"] - x_mean) / x_std
    train_y = (train["y"] - y_mean) / y_std
    torch.manual_seed(seed)
    core = WrenchRegressor(train_x.shape[1], hidden)
    optimizer = torch.optim.AdamW(core.parameters(), lr=1.0e-3, weight_decay=1.0e-5)
    sampler = sampler_for(train, sampling, seed)
    loader = DataLoader(
        TensorDataset(torch.from_numpy(train_x), torch.from_numpy(train_y)),
        batch_size=1024,
        shuffle=sampler is None,
        sampler=sampler,
        generator=torch.Generator().manual_seed(seed),
    )
    best_state, best_score, best_epoch, stale = None, math.inf, 0, 0
    for epoch in range(1, SCREEN_EPOCHS + 1):
        core.train()
        for batch_x, batch_y in loader:
            optimizer.zero_grad(set_to_none=True)
            loss = loss_value(core(batch_x) - batch_y, loss_kind)
            loss.backward()
            optimizer.step()
        model = make_normalized_model(core, x_mean, x_std, y_mean, y_std)
        prediction = training.predict_batches(model, validation["x"])
        score = error_metrics(validation["y"], prediction)["force_norm_rmse_n"]
        if score + 1.0e-7 < best_score:
            best_state = {key: value.detach().cpu().clone() for key, value in core.state_dict().items()}
            best_score, best_epoch, stale = score, epoch, 0
        else:
            stale += 1
        if stale >= 10:
            break
    if best_state is None:
        raise RuntimeError("training produced no checkpoint")
    core.load_state_dict(best_state)
    model = make_normalized_model(core, x_mean, x_std, y_mean, y_std)
    prediction = training.predict_batches(model, validation["x"])
    metrics, by_group = metrics_from_prediction(validation, prediction)
    return {
        "model": model,
        "prediction": prediction,
        "best_epoch": best_epoch,
        "metrics": metrics,
        "by_group": by_group,
    }


def ridge_candidate(train, validation, regularization):
    train_x = np.asarray(train["x"], dtype=np.float64)
    train_y = np.asarray(train["y"], dtype=np.float64)
    validation_x = np.asarray(validation["x"], dtype=np.float64)
    x_mean = train_x.mean(axis=0)
    x_std = np.maximum(train_x.std(axis=0), 1.0e-6)
    y_mean = train_y.mean(axis=0)
    y_std = np.maximum(train_y.std(axis=0), 1.0e-6)
    x = (train_x - x_mean) / x_std
    y = (train_y - y_mean) / y_std
    matrix = x.T @ x
    matrix.flat[:: len(matrix) + 1] += regularization
    coefficients = np.linalg.solve(matrix, x.T @ y)
    prediction = ((validation_x - x_mean) / x_std) @ coefficients * y_std + y_mean
    metrics, by_group = metrics_from_prediction(validation, prediction)
    return {
        "metrics": metrics,
        "by_group": by_group,
        "prediction": prediction,
        "coefficients": coefficients,
        "x_mean": x_mean,
        "x_std": x_std,
        "y_mean": y_mean,
        "y_std": y_std,
    }


def ridge_torch_model(result):
    coefficients = np.asarray(result["coefficients"], dtype=np.float32)
    core = WrenchRegressor(coefficients.shape[0], ())
    with torch.no_grad():
        core.layers[0].weight.copy_(torch.from_numpy(coefficients.T))
        core.layers[0].bias.zero_()
    return make_normalized_model(
        core,
        result["x_mean"],
        result["x_std"],
        result["y_mean"],
        result["y_std"],
    )


def zero_affine_correct(splits):
    train_sessions = splits["train"]
    x = np.column_stack(
        (np.ones(len(train_sessions)), np.stack([zero_force(row) for row in train_sessions]))
    )
    y = []
    for session in train_sessions:
        acceleration = np.linalg.norm(session.features[:, 12:18], axis=1)
        quiet = acceleration <= np.percentile(acceleration, 25.0)
        y.append(np.median(session.wrench[quiet, :3], axis=0))
    y = np.stack(y)

    def fit(rows_x, rows_y, regularization):
        penalty = np.eye(rows_x.shape[1]) * regularization
        penalty[0, 0] = 0.0
        return np.linalg.solve(rows_x.T @ rows_x + penalty, rows_x.T @ rows_y)

    scores = {}
    for regularization in (0.01, 0.1, 1.0, 10.0):
        predictions = []
        for index in range(len(x)):
            keep = np.arange(len(x)) != index
            predictions.append(x[index] @ fit(x[keep], y[keep], regularization))
        scores[regularization] = float(np.sqrt(np.mean((y - np.stack(predictions)) ** 2)))
    selected = min(scores, key=scores.get)
    coefficients = fit(x, y, selected)

    def correct(session):
        bias = np.zeros(6, dtype=np.float32)
        bias[:3] = np.r_[1.0, zero_force(session)] @ coefficients
        return replace(session, wrench=(session.wrench - bias).astype(np.float32))

    return {
        role: [correct(session) for session in sessions]
        for role, sessions in splits.items()
    }, {
        "selected_regularization": selected,
        "leave_one_group_out_rmse": scores,
        "coefficients": coefficients.tolist(),
    }


def serializable(result):
    return {
        "best_epoch": result.get("best_epoch"),
        "metrics": result["metrics"],
        "by_group": result["by_group"],
    }


def recurrent_screen(residual):
    name = "physical_residual_gru32"
    training.ABLATIONS[name] = ("sequence", 32, (128,), "gru")
    result = training.train_candidate(
        name,
        residual,
        epochs=30,
        batch_size=1024,
        learning_rate=1.0e-3,
        max_windows_per_session=5000,
        seed=SEED,
    )
    targets, predictions, groups = [], [], []
    offsets = np.arange(31, -1, -1)
    for group_index, session in enumerate(residual["validation"]):
        ends = np.arange(31, len(session.features))
        for start in range(0, len(ends), 8192):
            batch = ends[start : start + 8192]
            windows = session.features[batch[:, None] - offsets[None, :]]
            projected = targeted.diagnostic_projection(windows, "sequence")
            targets.append(session.wrench[batch])
            predictions.append(training.predict_batches(result["model"], projected))
            groups.append(np.full(len(batch), group_index, dtype=np.int16))
    validation = {
        "y": np.concatenate(targets),
        "group": np.concatenate(groups),
        "group_names": [row.group for row in residual["validation"]],
    }
    prediction = np.concatenate(predictions)
    metrics, by_group = metrics_from_prediction(validation, prediction)
    return {
        "best_epoch": result["best_epoch"],
        "train_cap_per_group": 5000,
        "metrics": metrics,
        "by_group": by_group,
    }


def regime_expert_screen(residual):
    train = build_data(residual["train"], "base", SEED, validation=False)
    validation = build_data(residual["validation"], "base", SEED + 1000, validation=True)
    low = train_mlp(
        subset_data(train, train["acceleration"] <= 4.0),
        subset_data(validation, validation["acceleration"] <= 4.0),
        (128, 128),
        "baseline",
        "random",
    )
    high = train_mlp(
        subset_data(train, train["acceleration"] >= 2.0),
        subset_data(validation, validation["acceleration"] >= 2.0),
        (128, 128),
        "baseline",
        "random",
    )
    low_prediction = training.predict_batches(low["model"], validation["x"])
    high_prediction = training.predict_batches(high["model"], validation["x"])
    weight = np.clip((validation["acceleration"] - 2.0) / 2.0, 0.0, 1.0)[:, None]
    prediction = low_prediction * (1.0 - weight) + high_prediction * weight
    metrics, by_group = metrics_from_prediction(validation, prediction)
    return {
        "low_best_epoch": low["best_epoch"],
        "high_best_epoch": high["best_epoch"],
        "metrics": metrics,
        "by_group": by_group,
    }


def self_check():
    values = np.arange(200 * 18, dtype=np.float32).reshape(200, 18) / 1000.0
    session = training.Session(
        Path("probe"),
        {"sample_hz": 262.5, "zero_verification": {"force_median_n": [1, 2, 3]}},
        values,
        np.zeros((200, 6), dtype=np.float32),
    )
    ends = np.asarray([127, 150])
    assert project_features(session, ends, "base").shape == (2, 24)
    assert project_features(session, ends, "zero").shape == (2, 27)
    assert project_features(session, ends, "elapsed").shape == (2, 25)
    assert project_features(session, ends, "short_multiscale").shape == (2, 54)
    assert project_features(session, ends, "multiscale").shape == (2, 78)
    assert np.allclose(project_features(session, ends, "zero")[:, -3:], [1, 2, 3])


def main():
    args = parse_args()
    self_check()
    if args.self_check:
        print("free-space improvement sweep self-check: PASS")
        return
    output = Path(args.output_dir).expanduser().resolve()
    if output.exists():
        raise FileExistsError(f"refusing to overwrite: {output}")
    started = time.monotonic()
    training.project_feature_windows = targeted.diagnostic_projection
    splits = targeted.load_fixed_splits()
    if set(splits) != {"train", "validation"}:
        raise RuntimeError("sweep refuses held-out/test splits")
    deltas = physical.gravity_deltas(splits)
    payload = physical.identify_payload(splits["train"], deltas)
    residual = physical.residualize(splits, deltas, payload)
    corrected, affine = zero_affine_correct(residual)

    data_cache = {}

    def data_for(kind, source=residual):
        key = (kind, id(source))
        if key not in data_cache:
            data_cache[key] = (
                build_data(source["train"], kind, SEED, validation=False),
                build_data(source["validation"], kind, SEED + 1000, validation=True),
            )
        return data_cache[key]

    candidates = {}
    for name, kind, hidden, loss_kind, sampling in CONFIGS:
        print(f"[SWEEP] {name}", flush=True)
        train, validation = data_for(kind)
        result = train_mlp(train, validation, hidden, loss_kind, sampling)
        candidates[name] = serializable(result)
        print(
            f"[SWEEP] {name}: max={result['metrics']['force_norm_max_n']:.4f} "
            f"p95={result['metrics']['force_norm_p95_n']:.4f} "
            f"rmse={result['metrics']['force_norm_rmse_n']:.4f}",
            flush=True,
        )

    print("[SWEEP] zero_median_affine", flush=True)
    train, validation = data_for("base", corrected)
    affine_result = train_mlp(train, validation, (128, 128), "baseline", "random")
    candidates["zero_median_affine"] = serializable(affine_result)

    for kind in ("base", "multiscale", "short_multiscale"):
        train, validation = data_for(kind)
        for regularization in (0.1, 1.0, 10.0):
            name = f"ridge_{kind}_{regularization:g}"
            print(f"[SWEEP] {name}", flush=True)
            result = ridge_candidate(train, validation, regularization)
            candidates[name] = serializable(result)

    data_cache.clear()

    print("[SWEEP] short multiscale seed stability and affine combination", flush=True)
    short_validation = build_data(
        residual["validation"],
        "short_multiscale",
        SEED + 1000,
        validation=True,
        common_history=32,
    )
    short_seed_results = {}
    short_predictions = []
    for seed in (7, 8, 9):
        short_train = build_data(
            residual["train"], "short_multiscale", seed, validation=False
        )
        result = ridge_candidate(short_train, short_validation, 1.0)
        short_seed_results[str(seed)] = serializable(result)
        short_predictions.append(result["prediction"])
    ensemble_prediction = np.mean(short_predictions, axis=0)
    ensemble_metrics, ensemble_by_group = metrics_from_prediction(
        short_validation, ensemble_prediction
    )

    corrected_short_validation = build_data(
        corrected["validation"],
        "short_multiscale",
        SEED + 1000,
        validation=True,
        common_history=32,
    )
    affine_short_results = {}
    for seed in (7, 8, 9):
        corrected_train = build_data(
            corrected["train"], "short_multiscale", seed, validation=False
        )
        affine_short_results[str(seed)] = serializable(
            ridge_candidate(corrected_train, corrected_short_validation, 1.0)
        )

    seed8_prediction = short_predictions[1]
    seed8_error = short_validation["y"] - seed8_prediction
    error_by_group = {
        name: seed8_error[short_validation["group"] == index]
        for index, name in enumerate(short_validation["group_names"])
    }
    finalist_filters = filtering.evaluate_sessions(error_by_group)

    print("[SWEEP] physical residual GRU32", flush=True)
    recurrent = recurrent_screen(residual)
    print("[SWEEP] motion regime experts", flush=True)
    regime_experts = regime_expert_screen(residual)

    selected_name = min(
        candidates,
        key=lambda name: (
            candidates[name]["metrics"]["force_norm_max_n"],
            candidates[name]["metrics"]["force_norm_p95_n"],
            candidates[name]["metrics"]["force_norm_rmse_n"],
        ),
    )
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip(),
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope": "train13 fit and development validation3 screen only; held-out/test not loaded",
        "approved": False,
        "runtime_or_acceptance_changed": False,
        "common_validation_history_samples": COMMON_HISTORY,
        "screen_seed": SEED,
        "settings": {"epochs": SCREEN_EPOCHS, "max_windows_per_session": MAX_WINDOWS},
        "manifest": training.session_manifest(splits),
        "physical_identification": payload,
        "zero_median_affine": affine,
        "candidates": candidates,
        "selected_screen_candidate": selected_name,
        "finalists": {
            "ridge_short_multiscale_seeds": short_seed_results,
            "ridge_short_multiscale_seed_mean": {
                "metrics": ensemble_metrics,
                "by_group": ensemble_by_group,
            },
            "zero_affine_plus_ridge_short_multiscale_seeds": affine_short_results,
            "physical_residual_gru32": recurrent,
            "motion_regime_experts": regime_experts,
        },
        "selected_offline_method": "ridge_short_multiscale_seed8",
        "selected_offline_metrics": short_seed_results["8"]["metrics"],
        "selected_offline_filters": finalist_filters,
        "warning": "development screen only; no model bundle was written",
        "elapsed_s": time.monotonic() - started,
    }
    output.mkdir(parents=True)
    (output / "improvement_sweep_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    (output / "split_manifest.json").write_text(
        json.dumps(report["manifest"], indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"[DONE] selected={selected_name} output={output}", flush=True)


if __name__ == "__main__":
    main()
