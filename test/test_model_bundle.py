import json
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory

from ft_fb_leaderarm.contract import (
    APPROVAL_CONTRACT,
    BASE_FEATURE_DIM,
    DEFAULT_ZERO_POSE_DEG,
    SAMPLE_HZ,
    SCHEMA_VERSION,
)
from ft_fb_leaderarm import contract
from ft_fb_leaderarm.model import (
    BundlePredictor,
    PHYSICAL_RIDGE_ABLATION,
    PHYSICAL_RIDGE_CONTRACT,
    RecurrentWrenchRegressor,
    WrenchRegressor,
    file_sha256,
    make_normalized_model,
    save_bundle,
)


def test_approved_bundle_round_trip(tmp_path):
    core = WrenchRegressor(12, ())
    for parameter in core.parameters():
        parameter.data.zero_()
    model = make_normalized_model(
        core,
        np.zeros(12),
        np.ones(12),
        np.zeros(6),
        np.ones(6),
    )
    metadata = {
        "schema_version": SCHEMA_VERSION,
        "approval_contract": APPROVAL_CONTRACT,
        "approved": True,
        "sample_hz": SAMPLE_HZ,
        "base_feature_dim": BASE_FEATURE_DIM,
        "ablation": "static_linear",
        "feature_mode": "static",
        "history": 1,
        "architecture": "mlp",
    }
    save_bundle(model, metadata, tmp_path)
    predictor = BundlePredictor(tmp_path)
    assert np.allclose(
        predictor.predict(np.zeros((1, BASE_FEATURE_DIM), dtype=np.float32)),
        0.0,
    )
    saved = json.loads((tmp_path / "metadata.json").read_text())
    assert len(saved["model_sha256"]) == 64


def test_rejected_bundle_is_not_runtime_loadable(tmp_path):
    core = WrenchRegressor(12, ())
    model = make_normalized_model(
        core,
        np.zeros(12),
        np.ones(12),
        np.zeros(6),
        np.ones(6),
    )
    save_bundle(
        model,
        {
            "schema_version": SCHEMA_VERSION,
            "approved": False,
            "sample_hz": SAMPLE_HZ,
            "base_feature_dim": BASE_FEATURE_DIM,
            "ablation": "static_linear",
            "feature_mode": "static",
            "history": 1,
            "architecture": "mlp",
        },
        tmp_path,
    )
    try:
        BundlePredictor(tmp_path)
    except RuntimeError as exc:
        assert "rejected" in str(exc)
    else:
        raise AssertionError("rejected model was accepted")


def test_exact_operator_selected_bundle_is_runtime_loadable(tmp_path, monkeypatch):
    core = WrenchRegressor(12, ())
    model = make_normalized_model(
        core,
        np.zeros(12),
        np.ones(12),
        np.zeros(6),
        np.ones(6),
    )
    save_bundle(
        model,
        {
            "schema_version": SCHEMA_VERSION,
            "approved": False,
            "sample_hz": SAMPLE_HZ,
            "base_feature_dim": BASE_FEATURE_DIM,
            "ablation": "static_linear",
            "feature_mode": "static",
            "history": 1,
            "architecture": "mlp",
        },
        tmp_path,
    )
    monkeypatch.setattr(
        contract,
        "OPERATOR_SELECTED_MODEL_SHA256",
        file_sha256(tmp_path / "model.ts"),
    )
    monkeypatch.setattr(
        contract,
        "OPERATOR_SELECTED_METADATA_SHA256",
        file_sha256(tmp_path / "metadata.json"),
    )
    predictor = BundlePredictor(tmp_path)
    assert predictor.acceptance_source == contract.OPERATOR_SELECTED_MODEL_CONTRACT


def test_lstm_and_gru_bundles_round_trip(tmp_path):
    for cell_type in ("lstm", "gru"):
        output = tmp_path / cell_type
        core = RecurrentWrenchRegressor(24, 16, cell_type)
        for parameter in core.parameters():
            parameter.data.zero_()
        model = make_normalized_model(
            core,
            np.zeros(24),
            np.ones(24),
            np.zeros(6),
            np.ones(6),
        )
        save_bundle(
            model,
            {
                "schema_version": SCHEMA_VERSION,
                "approval_contract": APPROVAL_CONTRACT,
                "approved": True,
                "sample_hz": SAMPLE_HZ,
                "base_feature_dim": BASE_FEATURE_DIM,
                "ablation": f"history_{cell_type}",
                "feature_mode": "sequence",
                "history": 16,
                "architecture": cell_type,
            },
            output,
        )
        predictor = BundlePredictor(output)
        assert np.allclose(
            predictor.predict(np.zeros((16, BASE_FEATURE_DIM), dtype=np.float32)),
            0.0,
        )


def test_obsolete_approval_contract_is_diagnostic_only(tmp_path):
    core = WrenchRegressor(12, ())
    model = make_normalized_model(
        core,
        np.zeros(12),
        np.ones(12),
        np.zeros(6),
        np.ones(6),
    )
    save_bundle(
        model,
        {
            "schema_version": SCHEMA_VERSION,
            "approved": True,
            "sample_hz": SAMPLE_HZ,
            "base_feature_dim": BASE_FEATURE_DIM,
            "ablation": "static_linear",
            "feature_mode": "static",
            "history": 1,
            "architecture": "mlp",
        },
        tmp_path,
    )
    BundlePredictor(tmp_path, require_approved=False)
    try:
        BundlePredictor(tmp_path)
    except RuntimeError as exc:
        assert "approval contract" in str(exc)
    else:
        raise AssertionError("obsolete approval contract was accepted")


def test_rejected_physical_ridge_bundle_is_diagnostic_loadable(tmp_path):
    core = WrenchRegressor(54, ())
    for parameter in core.parameters():
        parameter.data.zero_()
    model = make_normalized_model(
        core,
        np.zeros(54),
        np.ones(54),
        np.zeros(6),
        np.ones(6),
    )
    package = "aidin_dsr_dualarm_description"
    relative = "urdf/aidin_dsr_dualarm_aligned_hand.urdf"
    urdf = Path(get_package_share_directory(package)) / relative
    save_bundle(
        model,
        {
            "schema_version": SCHEMA_VERSION,
            "approved": False,
            "sample_hz": SAMPLE_HZ,
            "base_feature_dim": BASE_FEATURE_DIM,
            "ablation": PHYSICAL_RIDGE_ABLATION,
            "feature_mode": "short_multiscale",
            "history": 32,
            "architecture": "ridge",
            "prediction_contract": PHYSICAL_RIDGE_CONTRACT,
            "zero_pose_deg": list(DEFAULT_ZERO_POSE_DEG),
            "gravity_model": {
                "urdf_package": package,
                "urdf_relative_path": relative,
                "urdf_sha256": file_sha256(urdf),
                "frame": "right_link_6",
                "joint_names": [f"right_joint_{index}" for index in range(1, 7)],
                "mass_kg": 1.0,
                "com_sensor_m": [0.0, 0.1, 0.0],
            },
        },
        tmp_path,
    )
    predictor = BundlePredictor(tmp_path, require_approved=False)
    window = np.zeros((32, BASE_FEATURE_DIM), dtype=np.float32)
    window[:, :6] = np.deg2rad(DEFAULT_ZERO_POSE_DEG)
    assert np.allclose(predictor.predict(window), 0.0, atol=1.0e-6)
    window[-1, :6] += 0.2
    assert np.linalg.norm(predictor.predict(window)[:3]) > 0.1

    metadata_path = tmp_path / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["gravity_model"]["urdf_relative_path"] = "../../outside.urdf"
    metadata_path.write_text(json.dumps(metadata))
    try:
        BundlePredictor(tmp_path, require_approved=False)
    except RuntimeError as exc:
        assert "URDF" in str(exc)
    else:
        raise AssertionError("gravity URDF path escaped its package")
