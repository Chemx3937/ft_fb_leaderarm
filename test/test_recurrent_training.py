from pathlib import Path

import numpy as np

from ft_fb_leaderarm.train_ablation import Session, train_candidate


def test_lstm_and_gru_training_paths_accept_causal_windows():
    rng = np.random.default_rng(4)
    features = rng.normal(size=(48, 18)).astype(np.float32)
    wrench = rng.normal(scale=0.1, size=(48, 6)).astype(np.float32)
    session = Session(
        Path("synthetic.npz"),
        {"zero_set_id": "synthetic"},
        features,
        wrench,
    )
    splits = {"train": [session], "validation": [session]}
    for name in ("history_lstm", "history_gru"):
        result = train_candidate(
            name,
            splits,
            epochs=1,
            batch_size=16,
            learning_rate=1.0e-3,
            max_windows_per_session=32,
            seed=3,
        )
        assert result["architecture"] in ("lstm", "gru")
        assert result["validation"]["samples"] == 33
