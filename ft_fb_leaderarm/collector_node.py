#!/usr/bin/env python3
"""Collect contact-free right-arm state and physical FT data at 262.5 Hz."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from contact_observer_msgs.msg import ObserverInput
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .contract import (
    BASE_FEATURE_DIM,
    CausalFeatureBuilder,
    DEFAULT_ZERO_POSE_DEG,
    FixedPoseZeroVerifier,
    RateGate,
    SAMPLE_HZ,
    SCHEMA_VERSION,
    episode_timing_quality,
    finite_vector,
    stamp_to_seconds,
    wrench_from_message,
)


def sensor_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=4,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class PhysicalFtCollector(Node):
    def __init__(self):
        super().__init__("ft_free_space_collector")
        defaults = {
            "observer_input_topic": "/contact_state/observer_input",
            "ft_topic": "/aft_sensor2/wrench",
            "observer_input_frame": "right_base_link",
            "ft_frame": "aft_sensor2",
            "output_dir": str(Path.home() / ".ros/ft_fb_leaderarm/data"),
            "episode_prefix": "right_free_space",
            "sample_hz": SAMPLE_HZ,
            "max_sync_error_ms": 3.0,
            "max_source_age_ms": 20.0,
            "max_record_gap_ms": 10.0,
            "max_duration_s": 300.0,
            "minimum_episode_s": 10.0,
            "auto_start": False,
            "zero_set_confirmed": False,
            "zero_set_id": "",
            "payload_id": "",
            "controller_config_hash": "",
            "zero_pose_deg": list(DEFAULT_ZERO_POSE_DEG),
            "zero_pose_tolerance_deg": 1.0,
            "zero_max_joint_speed_rad_s": 0.02,
            "zero_settle_s": 1.0,
            "zero_force_norm_max_n": 1.0,
            "zero_force_axis_std_max_n": 0.40,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        sample_hz = float(self.get_parameter("sample_hz").value)
        if abs(sample_hz - SAMPLE_HZ) > 1.0e-9:
            raise RuntimeError(f"sample_hz is fixed at {SAMPLE_HZ} Hz")
        self.sample_hz = sample_hz
        self.max_sync_error_s = (
            float(self.get_parameter("max_sync_error_ms").value) * 1.0e-3
        )
        self.max_source_age_s = (
            float(self.get_parameter("max_source_age_ms").value) * 1.0e-3
        )
        self.max_record_gap_s = (
            float(self.get_parameter("max_record_gap_ms").value) * 1.0e-3
        )
        self.max_duration_s = float(self.get_parameter("max_duration_s").value)
        self.minimum_episode_s = float(
            self.get_parameter("minimum_episode_s").value
        )
        self.expected_observer_frame = str(
            self.get_parameter("observer_input_frame").value
        )
        self.expected_ft_frame = str(self.get_parameter("ft_frame").value)
        self.output_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.zero_verifier = FixedPoseZeroVerifier(
            zero_pose_deg=self.get_parameter("zero_pose_deg").value,
            pose_tolerance_deg=float(
                self.get_parameter("zero_pose_tolerance_deg").value
            ),
            max_joint_speed_rad_s=float(
                self.get_parameter("zero_max_joint_speed_rad_s").value
            ),
            settle_s=float(self.get_parameter("zero_settle_s").value),
            force_norm_max_n=float(
                self.get_parameter("zero_force_norm_max_n").value
            ),
            force_axis_std_max_n=float(
                self.get_parameter("zero_force_axis_std_max_n").value
            ),
        )
        self.rate_gate = RateGate(self.sample_hz)
        self.feature_builder = CausalFeatureBuilder()
        self.latest_state = None
        self.collecting = False
        self.auto_start_used = False
        self.episode_start_monotonic_s = 0.0
        self.episode_start_utc = ""
        self.last_saved_path = ""
        self.reset_episode()

        qos = sensor_qos()
        self.create_subscription(
            ObserverInput,
            str(self.get_parameter("observer_input_topic").value),
            self.state_callback,
            qos,
        )
        self.create_subscription(
            WrenchStamped,
            str(self.get_parameter("ft_topic").value),
            self.ft_callback,
            qos,
        )
        self.diagnostics_publisher = self.create_publisher(
            String, "~/diagnostics", 10
        )
        self.create_service(Trigger, "~/start_episode", self.start_callback)
        self.create_service(Trigger, "~/stop_episode", self.stop_callback)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f"physical FT collector ready: right arm, {self.sample_hz:.1f} Hz, "
            f"FT={self.get_parameter('ft_topic').value}, output={self.output_dir}"
        )

    def reset_episode(self):
        self.rows = {
            "stamp_s": [],
            "robot_stamp_s": [],
            "source_sequence": [],
            "features": [],
            "raw_wrench": [],
            "current_pose": [],
            "task_error": [],
            "sync_error_ms": [],
        }
        self.ft_callbacks = 0
        self.sync_rejections = 0
        self.invalid_rejections = 0
        self.duplicate_state_rejections = 0
        self.last_recorded_source_sequence = None

    def state_callback(self, msg):
        q = finite_vector(msg.q_rad, 6)
        dq = finite_vector(msg.dq_rad_s, 6)
        pose = finite_vector(msg.current_pose, 6)
        task_error = finite_vector(msg.task_error, 6)
        stamp_s = stamp_to_seconds(msg.header.stamp)
        frame_ok = msg.header.frame_id == self.expected_observer_frame
        valid = (
            bool(msg.valid)
            and frame_ok
            and stamp_s > 0.0
            and q is not None
            and dq is not None
            and pose is not None
            and task_error is not None
        )
        self.latest_state = {
            "valid": valid,
            "stamp_s": stamp_s,
            "receive_monotonic_s": time.monotonic(),
            "source_sequence": int(msg.source_sequence),
            "q": q,
            "dq": dq,
            "current_pose": pose,
            "task_error": task_error,
        }

    def ft_callback(self, msg):
        self.ft_callbacks += 1
        wrench = wrench_from_message(msg)
        ft_stamp_s = stamp_to_seconds(msg.header.stamp)
        now_monotonic_s = time.monotonic()
        state = self.latest_state
        frame_ok = msg.header.frame_id == self.expected_ft_frame
        pair_valid = (
            state is not None
            and state["valid"]
            and frame_ok
            and ft_stamp_s > 0.0
            and np.isfinite(wrench).all()
            and abs(ft_stamp_s - state["stamp_s"]) <= self.max_sync_error_s
            and now_monotonic_s - state["receive_monotonic_s"]
            <= self.max_source_age_s
        )
        if pair_valid:
            zero_ready = self.zero_verifier.update(
                now_monotonic_s, state["q"], state["dq"], wrench
            )
        else:
            self.zero_verifier.reset("invalid_or_unsynchronized_stream")
            zero_ready = False

        if (
            not self.collecting
            and not self.auto_start_used
            and bool(self.get_parameter("auto_start").value)
            and zero_ready
        ):
            ok, message = self.start_episode()
            self.auto_start_used = ok
            if ok:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)

        if not self.collecting:
            return
        if not pair_valid:
            self.sync_rejections += 1
            return
        if not self.rate_gate.accept(ft_stamp_s):
            return
        if state["source_sequence"] == self.last_recorded_source_sequence:
            self.duplicate_state_rejections += 1
            return
        feature = self.feature_builder.build(
            state["q"], state["dq"], state["stamp_s"]
        )
        if feature is None or feature.shape != (BASE_FEATURE_DIM,):
            self.invalid_rejections += 1
            return

        self.rows["stamp_s"].append(ft_stamp_s)
        self.rows["robot_stamp_s"].append(state["stamp_s"])
        self.rows["source_sequence"].append(state["source_sequence"])
        self.rows["features"].append(feature.astype(np.float32))
        self.rows["raw_wrench"].append(wrench.astype(np.float32))
        self.rows["current_pose"].append(state["current_pose"].astype(np.float32))
        self.rows["task_error"].append(state["task_error"].astype(np.float32))
        self.rows["sync_error_ms"].append(
            1000.0 * abs(ft_stamp_s - state["stamp_s"])
        )
        self.last_recorded_source_sequence = state["source_sequence"]

        if now_monotonic_s - self.episode_start_monotonic_s >= self.max_duration_s:
            path, accepted = self.finish_episode("max_duration")
            self.get_logger().info(
                f"episode auto-saved: {path} accepted={accepted}"
            )

    def start_episode(self):
        if self.collecting:
            return False, "episode is already collecting"
        if not bool(self.get_parameter("zero_set_confirmed").value):
            return False, "set zero_set_confirmed:=true only after AFT zero-set"
        zero_set_id = str(self.get_parameter("zero_set_id").value).strip()
        if not zero_set_id:
            return False, "zero_set_id is required for every hardware zero-set"
        if not str(self.get_parameter("payload_id").value).strip():
            return False, "payload_id is required to prevent mixed payload data"
        if not str(
            self.get_parameter("controller_config_hash").value
        ).strip():
            return False, (
                "controller_config_hash is required to prevent mixed controller data"
            )
        zero_ready, reason = self.zero_verifier.status()
        if not zero_ready:
            return False, f"fixed-pose zero verification failed: {reason}"
        self.reset_episode()
        self.rate_gate.reset()
        self.feature_builder.reset()
        self.episode_start_monotonic_s = time.monotonic()
        self.episode_start_utc = datetime.now(timezone.utc).isoformat()
        self.zero_snapshot = self.zero_verifier.summary()
        self.collecting = True
        return True, (
            "episode started; move only in confirmed free space and call "
            "~/stop_episode before making contact"
        )

    def start_callback(self, _request, response):
        response.success, response.message = self.start_episode()
        return response

    def stop_callback(self, _request, response):
        if not self.collecting:
            response.success = False
            response.message = "no active episode"
            return response
        path, accepted = self.finish_episode("service_stop")
        response.success = True
        response.message = f"saved {path}; training_accepted={accepted}"
        return response

    def finish_episode(self, stop_reason):
        self.collecting = False
        quality = episode_timing_quality(
            self.rows["stamp_s"],
            sample_hz=self.sample_hz,
            minimum_duration_s=self.minimum_episode_s,
            max_record_gap_s=self.max_record_gap_s,
            invalid_samples=self.invalid_rejections,
        )
        count = quality["samples"]
        duration_s = quality["duration_s"]
        actual_hz = quality["actual_hz"]
        max_record_gap_s = quality["max_record_gap_s"]
        accepted = quality["accepted"]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefix = str(self.get_parameter("episode_prefix").value).strip()
        base = self.output_dir / f"{prefix}_{stamp}"
        suffix = 1
        while base.with_suffix(".npz").exists():
            base = self.output_dir / f"{prefix}_{stamp}_{suffix:02d}"
            suffix += 1

        metadata = {
            "schema_version": SCHEMA_VERSION,
            "created_utc": self.episode_start_utc,
            "stopped_utc": datetime.now(timezone.utc).isoformat(),
            "stop_reason": str(stop_reason),
            "accepted": accepted,
            "free_space_only": True,
            "robot_side": "right",
            "sample_hz": self.sample_hz,
            "samples": count,
            "duration_s": duration_s,
            "actual_hz": actual_hz,
            "max_record_gap_ms": max_record_gap_s * 1000.0,
            "max_record_gap_limit_ms": self.max_record_gap_s * 1000.0,
            "observer_input_topic": str(
                self.get_parameter("observer_input_topic").value
            ),
            "ft_topic": str(self.get_parameter("ft_topic").value),
            "observer_input_frame": self.expected_observer_frame,
            "ft_frame": self.expected_ft_frame,
            "feature_contract": "q_rad[6],dq_rad_s[6],causal_qdd_rad_s2[6]",
            "target_contract": "physical_ft_sensor_frame_[Fx,Fy,Fz,Mx,My,Mz]",
            "zero_set_id": str(self.get_parameter("zero_set_id").value),
            "zero_pose_deg": list(self.get_parameter("zero_pose_deg").value),
            "zero_verification": self.zero_snapshot,
            "payload_id": str(self.get_parameter("payload_id").value),
            "controller_config_hash": str(
                self.get_parameter("controller_config_hash").value
            ),
            "ft_callbacks": self.ft_callbacks,
            "sync_rejections": self.sync_rejections,
            "invalid_rejections": self.invalid_rejections,
            "duplicate_state_rejections": self.duplicate_state_rejections,
            "contact_free_operator_contract": (
                "operator stopped this episode before any physical contact"
            ),
        }
        arrays = {
            "stamp_s": np.asarray(self.rows["stamp_s"], dtype=np.float64),
            "robot_stamp_s": np.asarray(
                self.rows["robot_stamp_s"], dtype=np.float64
            ),
            "source_sequence": np.asarray(
                self.rows["source_sequence"], dtype=np.uint64
            ),
            "features": np.asarray(
                self.rows["features"], dtype=np.float32
            ).reshape((-1, BASE_FEATURE_DIM)),
            "raw_wrench": np.asarray(
                self.rows["raw_wrench"], dtype=np.float32
            ).reshape((-1, 6)),
            "current_pose": np.asarray(
                self.rows["current_pose"], dtype=np.float32
            ).reshape((-1, 6)),
            "task_error": np.asarray(
                self.rows["task_error"], dtype=np.float32
            ).reshape((-1, 6)),
            "sync_error_ms": np.asarray(
                self.rows["sync_error_ms"], dtype=np.float32
            ),
            "metadata": np.asarray(json.dumps(metadata, sort_keys=True)),
        }
        target = base.with_suffix(".npz")
        temporary = base.with_suffix(".tmp.npz")
        np.savez_compressed(temporary, **arrays)
        os.replace(temporary, target)
        base.with_suffix(".json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
        )
        self.last_saved_path = str(target)
        self.reset_episode()
        self.rate_gate.reset()
        self.feature_builder.reset()
        return target, accepted

    def publish_diagnostics(self):
        message = String()
        message.data = json.dumps(
            {
                "collecting": self.collecting,
                "samples": len(self.rows["stamp_s"]),
                "zero": self.zero_verifier.summary(),
                "last_saved_path": self.last_saved_path,
                "sync_rejections": self.sync_rejections,
                "invalid_rejections": self.invalid_rejections,
            },
            sort_keys=True,
        )
        self.diagnostics_publisher.publish(message)

    def destroy_node(self):
        if self.collecting and self.rows["stamp_s"]:
            path, accepted = self.finish_episode("node_shutdown")
            self.get_logger().warn(
                f"active episode saved during shutdown: {path} accepted={accepted}"
            )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalFtCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        previous_sigint = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)


if __name__ == "__main__":
    main()
