"""Small Torch regressors and a sealed runtime bundle loader."""

import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from .contract import (
    ABLATIONS,
    BASE_FEATURE_DIM,
    SAMPLE_HZ,
    SCHEMA_VERSION,
    project_feature_windows,
)


class WrenchRegressor(nn.Module):
    def __init__(self, input_dim, hidden_dims):
        super().__init__()
        dimensions = (int(input_dim),) + tuple(int(v) for v in hidden_dims) + (6,)
        layers = []
        for index, (source, target) in enumerate(
            zip(dimensions[:-1], dimensions[1:])
        ):
            layers.append(nn.Linear(source, target))
            if index < len(dimensions) - 2:
                layers.append(nn.SiLU())
        self.layers = nn.Sequential(*layers)

    def forward(self, values):
        return self.layers(values)


class RecurrentWrenchRegressor(nn.Module):
    def __init__(self, input_dim, hidden_dim, cell_type):
        super().__init__()
        if cell_type == "lstm":
            self.recurrent = nn.LSTM(input_dim, hidden_dim, batch_first=True)
        elif cell_type == "gru":
            self.recurrent = nn.GRU(input_dim, hidden_dim, batch_first=True)
        else:
            raise ValueError(f"unsupported recurrent cell: {cell_type}")
        self.head = nn.Linear(hidden_dim, 6)

    def forward(self, values):
        sequence, _ = self.recurrent(values)
        return self.head(sequence[:, -1, :])


class NormalizedWrenchModel(nn.Module):
    def __init__(self, core, x_mean, x_std, y_mean, y_std):
        super().__init__()
        self.core = core
        self.register_buffer(
            "x_mean", torch.as_tensor(x_mean, dtype=torch.float32)
        )
        self.register_buffer(
            "x_std", torch.as_tensor(x_std, dtype=torch.float32)
        )
        self.register_buffer(
            "y_mean", torch.as_tensor(y_mean, dtype=torch.float32)
        )
        self.register_buffer(
            "y_std", torch.as_tensor(y_std, dtype=torch.float32)
        )

    def forward(self, values):
        normalized = (values - self.x_mean) / self.x_std
        return self.core(normalized) * self.y_std + self.y_mean


def make_normalized_model(core, x_mean, x_std, y_mean, y_std):
    return NormalizedWrenchModel(
        core,
        np.asarray(x_mean, dtype=np.float32),
        np.asarray(x_std, dtype=np.float32),
        np.asarray(y_mean, dtype=np.float32),
        np.asarray(y_std, dtype=np.float32),
    )


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save_bundle(model, metadata, output_dir):
    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = output_dir / "model.ts"
    temporary = output_dir / "model.tmp.ts"
    scripted = torch.jit.script(model.eval().cpu())
    torch.jit.save(scripted, str(temporary))
    temporary.replace(model_path)
    document = dict(metadata)
    document["model_sha256"] = file_sha256(model_path)
    (output_dir / "metadata.json").write_text(
        json.dumps(document, indent=2, sort_keys=True), encoding="utf-8"
    )
    return model_path


class BundlePredictor:
    def __init__(self, model_path, require_approved=True):
        model_path = Path(model_path).expanduser().resolve()
        if model_path.is_dir():
            model_path = model_path / "model.ts"
        metadata_path = model_path.parent / "metadata.json"
        if not model_path.is_file() or not metadata_path.is_file():
            raise FileNotFoundError(
                f"model.ts and metadata.json are required under {model_path.parent}"
            )
        self.metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        if int(self.metadata.get("schema_version", -1)) != SCHEMA_VERSION:
            raise RuntimeError("unsupported model schema")
        if abs(float(self.metadata.get("sample_hz", 0.0)) - SAMPLE_HZ) > 1.0e-9:
            raise RuntimeError(f"model sample_hz must be {SAMPLE_HZ}")
        if int(self.metadata.get("base_feature_dim", -1)) != BASE_FEATURE_DIM:
            raise RuntimeError("model base feature contract is invalid")
        if require_approved and not bool(self.metadata.get("approved", False)):
            raise RuntimeError("model is rejected; the 1 N/runtime gates did not pass")
        expected_hash = str(self.metadata.get("model_sha256", ""))
        if not expected_hash or file_sha256(model_path) != expected_hash:
            raise RuntimeError("model SHA-256 does not match metadata")
        self.ablation = str(self.metadata.get("ablation", ""))
        if self.ablation not in ABLATIONS:
            raise RuntimeError(f"unsupported model ablation: {self.ablation}")
        expected_mode, expected_history, _, expected_architecture = ABLATIONS[
            self.ablation
        ]
        self.mode = str(self.metadata.get("feature_mode", ""))
        self.history = int(self.metadata.get("history", 0))
        self.architecture = str(self.metadata.get("architecture", ""))
        if (
            self.mode != expected_mode
            or self.history != expected_history
            or self.architecture != expected_architecture
        ):
            raise RuntimeError(
                "model ablation and feature/history/architecture metadata disagree"
            )
        self.model = torch.jit.load(str(model_path), map_location="cpu").eval()

    def predict(self, base_feature_window):
        window = np.asarray(base_feature_window, dtype=np.float32)
        if window.shape != (self.history, BASE_FEATURE_DIM):
            raise ValueError(
                f"feature window must have shape {(self.history, BASE_FEATURE_DIM)}"
            )
        projected = project_feature_windows(window[None, :, :], self.mode)
        with torch.inference_mode():
            result = self.model(torch.from_numpy(projected))
        output = result.detach().cpu().numpy().reshape(-1)
        if output.shape != (6,) or not np.isfinite(output).all():
            raise RuntimeError("model produced a non-finite or malformed wrench")
        return output.astype(np.float64)
