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


PHYSICAL_RIDGE_ABLATION = "physical_residual_short_multiscale_ridge"
PHYSICAL_RIDGE_CONTRACT = "physical_payload_gravity_plus_learned_residual"
RUNTIME_ABLATIONS = {
    **ABLATIONS,
    PHYSICAL_RIDGE_ABLATION: ("short_multiscale", 32, (), "ridge"),
}


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


class PayloadGravityModel:
    def __init__(self, metadata, zero_pose_deg):
        import pinocchio as pin
        from ament_index_python.packages import get_package_share_directory

        package = str(metadata.get("urdf_package", "")).strip()
        relative = str(metadata.get("urdf_relative_path", "")).strip()
        relative_path = Path(relative)
        package_root = Path(get_package_share_directory(package)).resolve()
        urdf = package_root / relative_path
        if (
            not relative
            or relative_path.is_absolute()
            or ".." in relative_path.parts
            or not urdf.is_file()
            or file_sha256(urdf) != metadata.get("urdf_sha256")
        ):
            raise RuntimeError(
                "payload gravity URDF is missing or differs from metadata"
            )
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        frame = str(metadata.get("frame", "")).strip()
        self.frame_id = self.model.getFrameId(frame)
        if self.frame_id >= len(self.model.frames):
            raise RuntimeError(f"payload gravity frame is missing: {frame}")
        joint_names = metadata.get("joint_names", [])
        if len(joint_names) != 6:
            raise RuntimeError("payload gravity requires six joint names")
        joint_ids = [self.model.getJointId(str(name)) for name in joint_names]
        if any(joint_id == 0 for joint_id in joint_ids):
            raise RuntimeError("payload gravity joint is missing from URDF")
        self.joint_indices = [
            self.model.joints[joint_id].idx_q for joint_id in joint_ids
        ]
        self.q_full = pin.neutral(self.model)
        self.mass_kg = float(metadata.get("mass_kg", 0.0))
        self.com_sensor_m = np.asarray(
            metadata.get("com_sensor_m", []), dtype=np.float64
        )
        zero_pose = np.deg2rad(np.asarray(zero_pose_deg, dtype=np.float64))
        if (
            not np.isfinite(self.mass_kg)
            or self.mass_kg <= 0.0
            or self.com_sensor_m.shape != (3,)
            or not np.isfinite(self.com_sensor_m).all()
            or zero_pose.shape != (6,)
            or not np.isfinite(zero_pose).all()
        ):
            raise RuntimeError("payload gravity parameters are invalid")
        self.gravity_at_zero = self._sensor_gravity(zero_pose)

    def _sensor_gravity(self, q_rad):
        self.q_full[self.joint_indices] = q_rad
        self.pin.forwardKinematics(self.model, self.data, self.q_full)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_id].rotation.T @ self.model.gravity.linear

    def predict(self, q_rad):
        q = np.asarray(q_rad, dtype=np.float64)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("payload gravity q must contain six finite values")
        force = self.mass_kg * (self._sensor_gravity(q) - self.gravity_at_zero)
        return np.concatenate((force, np.cross(self.com_sensor_m, force)))


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
        if self.ablation not in RUNTIME_ABLATIONS:
            raise RuntimeError(f"unsupported model ablation: {self.ablation}")
        expected_mode, expected_history, _, expected_architecture = (
            RUNTIME_ABLATIONS[self.ablation]
        )
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
        prediction_contract = str(
            self.metadata.get("prediction_contract", "")
        ).strip()
        self.gravity_model = None
        if prediction_contract == PHYSICAL_RIDGE_CONTRACT:
            self.gravity_model = PayloadGravityModel(
                self.metadata.get("gravity_model", {}),
                self.metadata.get("zero_pose_deg", []),
            )
        elif prediction_contract:
            raise RuntimeError(f"unsupported prediction contract: {prediction_contract}")

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
        if self.gravity_model is not None:
            output = output.astype(np.float64) + self.gravity_model.predict(
                window[-1, :6]
            )
        if output.shape != (6,) or not np.isfinite(output).all():
            raise RuntimeError("model produced a non-finite or malformed wrench")
        return output.astype(np.float64)
