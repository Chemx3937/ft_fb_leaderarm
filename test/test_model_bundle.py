import json

import numpy as np

from ft_fb_leaderarm.contract import BASE_FEATURE_DIM, SAMPLE_HZ, SCHEMA_VERSION
from ft_fb_leaderarm.model import (
    BundlePredictor,
    RecurrentWrenchRegressor,
    WrenchRegressor,
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
