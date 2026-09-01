#!/usr/bin/env python3
"""Run an accepted physical-FT model and publish canonical contact residuals."""

from collections import Counter, deque
import json
import math
import signal
import time

import numpy as np
import rclpy
import torch
from contact_observer_msgs.msg import ContactObservation, ObserverInput
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from .contract import (
    CausalFeatureBuilder,
    DEFAULT_ZERO_POSE_DEG,
    FixedPoseZeroVerifier,
    SAMPLE_HZ,
    SchmittContactDetector,
    fill_wrench_message,
    finite_vector,
    sensor_wrench_to_base,
    stamp_to_seconds,
    wrench_from_message,
)
from .model import BundlePredictor


def sensor_qos():
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=4,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class PhysicalFtContactObserver(Node):
    def __init__(self):
        super().__init__("ft_contact_observer")
        defaults = {
            "model_path": "",
            "observer_input_topic": "/contact_state/observer_input",
            "ft_topic": "/aft_sensor2/wrench",
            "contact_observation_topic": "/contact_observer/right/observation",
            "predicted_sensor_wrench_topic": (
                "/ft_free_space/right/predicted_wrench"
            ),
            "contact_sensor_wrench_topic": "/ft_free_space/right/contact_wrench",
            "diagnostics_topic": "~/diagnostics",
            "observer_input_frame": "right_base_link",
            "ft_frame": "aft_sensor2",
            "sample_hz": SAMPLE_HZ,
            "max_sync_error_ms": 3.0,
            "max_source_age_ms": 20.0,
            "clock_future_tolerance_ms": 2.0,
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
            "sensor_to_tip_zyx_deg": [0.0, 0.0, 0.0],
            "tip_to_sensor_translation_m": [0.0, 0.0, 0.0],
            "force_on_n": 2.5,
            "force_off_n": 1.2,
            "contact_hold_ms": 12.0,
            "free_hold_ms": 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            raise RuntimeError("model_path is required")
        if not bool(self.get_parameter("zero_set_confirmed").value):
            raise RuntimeError(
                "zero_set_confirmed must be true after hardware AFT zero-set"
            )
        if not str(self.get_parameter("zero_set_id").value).strip():
            raise RuntimeError("zero_set_id is required for the runtime zero-set")
        sample_hz = float(self.get_parameter("sample_hz").value)
        if abs(sample_hz - SAMPLE_HZ) > 1.0e-9:
            raise RuntimeError(f"sample_hz is fixed at {SAMPLE_HZ} Hz")

        torch.set_num_threads(1)
        self.predictor = BundlePredictor(model_path, require_approved=True)
        self.sample_hz = sample_hz
        if abs(self.predictor.sample_hz - self.sample_hz) > 1.0e-9:
            raise RuntimeError("observer and model sample_hz contracts differ")
        self.expected_observer_frame = str(
            self.get_parameter("observer_input_frame").value
        )
        self.expected_ft_frame = str(self.get_parameter("ft_frame").value)
        for parameter, metadata_key in (
            ("observer_input_frame", "observer_input_frame"),
            ("ft_frame", "ft_frame"),
            ("payload_id", "payload_id"),
            ("controller_config_hash", "controller_config_hash"),
        ):
            configured = str(self.get_parameter(parameter).value).strip()
            trained = str(self.predictor.metadata.get(metadata_key, "")).strip()
            if not configured or configured != trained:
                raise RuntimeError(
                    f"{parameter}={configured!r} does not match model {trained!r}"
                )
        configured_zero_pose = np.asarray(
            self.get_parameter("zero_pose_deg").value, dtype=np.float64
        )
        trained_zero_pose = np.asarray(
            self.predictor.metadata.get("zero_pose_deg", []), dtype=np.float64
        )
        if (
            configured_zero_pose.shape != (6,)
            or trained_zero_pose.shape != (6,)
            or not np.allclose(
                configured_zero_pose, trained_zero_pose, rtol=0.0, atol=1.0e-9
            )
        ):
            raise RuntimeError("zero_pose_deg does not match the model contract")
        self.prewarm_benchmark = self.prewarm_model(configured_zero_pose)

        self.max_sync_error_s = (
            float(self.get_parameter("max_sync_error_ms").value) * 1.0e-3
        )
        self.max_source_age_s = (
            float(self.get_parameter("max_source_age_ms").value) * 1.0e-3
        )
        self.future_tolerance_s = (
            float(self.get_parameter("clock_future_tolerance_ms").value) * 1.0e-3
        )
        self.sensor_to_tip_zyx_deg = finite_vector(
            self.get_parameter("sensor_to_tip_zyx_deg").value, 3
        )
        self.tip_to_sensor_translation_m = finite_vector(
            self.get_parameter("tip_to_sensor_translation_m").value, 3
        )
        if (
            self.sensor_to_tip_zyx_deg is None
            or self.tip_to_sensor_translation_m is None
        ):
            raise RuntimeError("sensor transform must contain finite 3-vectors")
        self.zero_verifier = FixedPoseZeroVerifier(
            zero_pose_deg=configured_zero_pose,
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
        self.zero_verified = False
        self.feature_builder = CausalFeatureBuilder()
        self.history = deque(maxlen=self.predictor.history)
        self.contact_detector = SchmittContactDetector(
            force_on_n=float(self.get_parameter("force_on_n").value),
            force_off_n=float(self.get_parameter("force_off_n").value),
            contact_hold_s=(
                float(self.get_parameter("contact_hold_ms").value) * 1.0e-3
            ),
            free_hold_s=(
                float(self.get_parameter("free_hold_ms").value) * 1.0e-3
            ),
        )
        self.latest_state = None
        self.latest_ft = None
        self.last_source_sequence = None
        self.prediction_sequence = 0
        self.cycles = 0
        self.valid_predictions = 0
        self.invalid_publications = 0
        self.inference_failures = 0
        self.deadline_misses = 0
        self.last_inference_ms = 0.0
        self.max_inference_ms = 0.0
        self.last_residual_force_norm_n = 0.0
        self.invalid_reason_counts = Counter()
        self.start_monotonic_s = time.monotonic()

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
        self.observation_publisher = self.create_publisher(
            ContactObservation,
            str(self.get_parameter("contact_observation_topic").value),
            qos,
        )
        self.prediction_publisher = self.create_publisher(
            WrenchStamped,
            str(self.get_parameter("predicted_sensor_wrench_topic").value),
            qos,
        )
        self.residual_publisher = self.create_publisher(
            WrenchStamped,
            str(self.get_parameter("contact_sensor_wrench_topic").value),
            qos,
        )
        self.diagnostics_publisher = self.create_publisher(
            String, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.create_timer(1.0 / self.sample_hz, self.inference_callback)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info(
            f"accepted model={model_path}, "
            f"contract={self.predictor.acceptance_source}, "
            f"ablation={self.predictor.ablation}, "
            f"history={self.predictor.history}, output={self.sample_hz:.1f} Hz, "
            f"prewarm_p99={self.prewarm_benchmark['p99_ms']:.3f} ms"
        )

    def prewarm_model(self, zero_pose_deg):
        feature = np.zeros(18, dtype=np.float32)
        feature[:6] = np.deg2rad(zero_pose_deg)
        window = np.repeat(
            feature[None, :], self.predictor.history, axis=0
        )
        for _ in range(20):
            self.predictor.predict(window)
        durations_ms = []
        for _ in range(200):
            start_ns = time.perf_counter_ns()
            self.predictor.predict(window)
            durations_ms.append(
                (time.perf_counter_ns() - start_ns) * 1.0e-6
            )
        values = np.asarray(durations_ms)
        result = {
            "calls": len(values),
            "p99_ms": float(np.percentile(values, 99.0)),
            "max_ms": float(np.max(values)),
        }
        period_ms = 1000.0 / self.sample_hz
        if result["p99_ms"] > 0.80 * period_ms or result["max_ms"] > period_ms:
            raise RuntimeError(
                f"runtime prewarm failed the {self.sample_hz:.1f} Hz deadline: "
                f"p99={result['p99_ms']:.3f} ms, "
                f"max={result['max_ms']:.3f} ms, period={period_ms:.3f} ms"
            )
        return result

    def state_callback(self, msg):
        q = finite_vector(msg.q_rad, 6)
        dq = finite_vector(msg.dq_rad_s, 6)
        current_pose = finite_vector(msg.current_pose, 6)
        stamp_s = stamp_to_seconds(msg.header.stamp)
        self.latest_state = {
            "valid": (
                bool(msg.valid)
                and msg.header.frame_id == self.expected_observer_frame
                and stamp_s > 0.0
                and q is not None
                and dq is not None
                and current_pose is not None
            ),
            "stamp_s": stamp_s,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "source_sequence": int(msg.source_sequence),
            "q": q,
            "dq": dq,
            "current_pose": current_pose,
            "receive_monotonic_s": time.monotonic(),
        }

    def ft_callback(self, msg):
        stamp_s = stamp_to_seconds(msg.header.stamp)
        wrench = wrench_from_message(msg)
        self.latest_ft = {
            "valid": (
                msg.header.frame_id == self.expected_ft_frame
                and stamp_s > 0.0
                and np.isfinite(wrench).all()
            ),
            "stamp_s": stamp_s,
            "stamp_sec": int(msg.header.stamp.sec),
            "stamp_nanosec": int(msg.header.stamp.nanosec),
            "wrench": wrench,
            "receive_monotonic_s": time.monotonic(),
        }

    def pair_status(self, now_ros_s, now_monotonic_s):
        state, ft = self.latest_state, self.latest_ft
        if state is None or ft is None:
            return False, "waiting_for_streams"
        if not state["valid"] or not ft["valid"]:
            return False, "invalid_input"
        state_age_s = now_ros_s - state["stamp_s"]
        ft_age_s = now_ros_s - ft["stamp_s"]
        if (
            state_age_s < -self.future_tolerance_s
            or ft_age_s < -self.future_tolerance_s
            or state_age_s > self.max_source_age_s
            or ft_age_s > self.max_source_age_s
        ):
            return False, "stale_or_future_input"
        if (
            now_monotonic_s - state["receive_monotonic_s"]
            > self.max_source_age_s
            or now_monotonic_s - ft["receive_monotonic_s"]
            > self.max_source_age_s
        ):
            return False, "locally_stale_input"
        if abs(state["stamp_s"] - ft["stamp_s"]) > self.max_sync_error_s:
            return False, "unsynchronized_input"
        if state["source_sequence"] == self.last_source_sequence:
            return False, "duplicate_robot_source"
        return True, "ok"

    def clear_causal_state(self):
        self.feature_builder.reset()
        self.history.clear()
        self.contact_detector.reset()

    def inference_callback(self):
        callback_start_ns = time.perf_counter_ns()
        self.cycles += 1
        now_message = self.get_clock().now().to_msg()
        now_ros_s = float(now_message.sec) + 1.0e-9 * float(now_message.nanosec)
        now_monotonic_s = time.monotonic()
        valid_pair, reason = self.pair_status(now_ros_s, now_monotonic_s)
        if not valid_pair:
            if reason != "duplicate_robot_source":
                self.clear_causal_state()
            self.publish_invalid(now_message, reason)
            return

        state, ft = self.latest_state, self.latest_ft
        if not self.zero_verified:
            self.zero_verified = self.zero_verifier.update(
                now_monotonic_s, state["q"], state["dq"], ft["wrench"]
            )
            if not self.zero_verified:
                self.publish_invalid(
                    now_message, self.zero_verifier.last_reason
                )
                return
            self.get_logger().info(
                "fixed-pose physical FT zero verification passed; "
                "free-space inference is now enabled"
            )
            self.clear_causal_state()

        feature = self.feature_builder.build(
            state["q"], state["dq"], state["stamp_s"]
        )
        self.last_source_sequence = state["source_sequence"]
        if feature is None:
            self.clear_causal_state()
            self.publish_invalid(now_message, "invalid_causal_feature")
            return
        self.history.append(feature.astype(np.float32))
        if len(self.history) < self.predictor.history:
            self.publish_invalid(now_message, "history_warmup")
            return

        inference_start_ns = time.perf_counter_ns()
        try:
            prediction_sensor = self.predictor.predict(
                np.stack(self.history, axis=0)
            )
            residual_sensor = ft["wrench"] - prediction_sensor
            prediction_base = sensor_wrench_to_base(
                prediction_sensor,
                state["current_pose"],
                self.sensor_to_tip_zyx_deg,
                self.tip_to_sensor_translation_m,
            )
            residual_base = sensor_wrench_to_base(
                residual_sensor,
                state["current_pose"],
                self.sensor_to_tip_zyx_deg,
                self.tip_to_sensor_translation_m,
            )
        except Exception as exc:
            self.inference_failures += 1
            self.clear_causal_state()
            self.publish_invalid(now_message, f"inference_failed:{exc}")
            return
        self.last_inference_ms = (
            time.perf_counter_ns() - inference_start_ns
        ) * 1.0e-6
        self.max_inference_ms = max(
            self.max_inference_ms, self.last_inference_ms
        )
        if self.last_inference_ms > 1000.0 / self.sample_hz:
            self.deadline_misses += 1

        force_norm_n = float(np.linalg.norm(residual_sensor[:3]))
        self.last_residual_force_norm_n = force_norm_n
        in_contact = self.contact_detector.update(force_norm_n, now_ros_s)
        self.prediction_sequence += 1
        observation = ContactObservation()
        observation.header.stamp.sec = state["stamp_sec"]
        observation.header.stamp.nanosec = state["stamp_nanosec"]
        observation.header.frame_id = self.expected_observer_frame
        observation.publish_stamp = now_message
        observation.source_sequence = state["source_sequence"]
        observation.prediction_sequence = self.prediction_sequence
        observation.contact_wrench = residual_base.tolist()
        observation.free_space_wrench_prediction = prediction_base.tolist()
        observation.contact_state = (
            ContactObservation.CONTACT if in_contact else ContactObservation.FREE
        )
        observation.contact_score = force_norm_n
        observation.valid = True
        observation.model_ready = True
        observation.prediction_age_ms = max(
            0.0, 1000.0 * (now_ros_s - state["stamp_s"])
        )
        observation.observer_latency_ms = max(
            0.0, 1000.0 * (now_ros_s - ft["stamp_s"])
        )
        self.observation_publisher.publish(observation)
        self.publish_sensor_wrenches(
            ft, prediction_sensor, residual_sensor
        )
        self.valid_predictions += 1
        callback_ms = (time.perf_counter_ns() - callback_start_ns) * 1.0e-6
        if callback_ms > 1000.0 / self.sample_hz:
            self.deadline_misses += 1

    def publish_invalid(self, now_message, reason):
        self.invalid_publications += 1
        self.invalid_reason_counts[str(reason).split(":", 1)[0]] += 1
        observation = ContactObservation()
        state = self.latest_state
        if state is not None and state["stamp_s"] > 0.0:
            observation.header.stamp.sec = state["stamp_sec"]
            observation.header.stamp.nanosec = state["stamp_nanosec"]
            observation.source_sequence = state["source_sequence"]
        else:
            observation.header.stamp = now_message
        observation.header.frame_id = self.expected_observer_frame
        observation.publish_stamp = now_message
        observation.prediction_sequence = self.prediction_sequence
        observation.contact_wrench = [0.0] * 6
        observation.free_space_wrench_prediction = [0.0] * 6
        observation.contact_state = ContactObservation.FREE
        observation.contact_score = 0.0
        observation.valid = False
        observation.model_ready = True
        observation.prediction_age_ms = -1.0
        observation.observer_latency_ms = -1.0
        self.observation_publisher.publish(observation)
        self.last_invalid_reason = str(reason)

    def publish_sensor_wrenches(self, ft, prediction, residual):
        for publisher, values in (
            (self.prediction_publisher, prediction),
            (self.residual_publisher, residual),
        ):
            message = WrenchStamped()
            message.header.stamp.sec = ft["stamp_sec"]
            message.header.stamp.nanosec = ft["stamp_nanosec"]
            message.header.frame_id = self.expected_ft_frame
            fill_wrench_message(message, values)
            publisher.publish(message)

    def publish_diagnostics(self):
        message = String()
        message.data = json.dumps(
            {
                "approved_model": True,
                "model_acceptance_source": self.predictor.acceptance_source,
                "model_ready": True,
                "baseline_ready": self.zero_verified,
                "observer_ready": (
                    self.zero_verified
                    and len(self.history) >= self.predictor.history
                    and self.valid_predictions > 0
                ),
                "residual_bias_calibration_enabled": False,
                "residual_bias_ready": True,
                "sample_hz": self.sample_hz,
                "uptime_s": time.monotonic() - self.start_monotonic_s,
                "model_sha256": self.predictor.metadata["model_sha256"],
                "zero_set_id": str(self.get_parameter("zero_set_id").value),
                "payload_id": str(self.get_parameter("payload_id").value),
                "controller_config_hash": str(
                    self.get_parameter("controller_config_hash").value
                ),
                "cycles": self.cycles,
                "valid_predictions": self.valid_predictions,
                "invalid_publications": self.invalid_publications,
                "invalid_reason_counts": dict(self.invalid_reason_counts),
                "zero_verified": self.zero_verified,
                "zero": self.zero_verifier.summary(),
                "history": len(self.history),
                "history_required": self.predictor.history,
                "last_invalid_reason": getattr(
                    self, "last_invalid_reason", ""
                ),
                "last_inference_ms": self.last_inference_ms,
                "max_inference_ms": self.max_inference_ms,
                "prewarm_benchmark": self.prewarm_benchmark,
                "deadline_misses": self.deadline_misses,
                "inference_failures": self.inference_failures,
                "residual_force_norm_n": self.last_residual_force_norm_n,
                "contact": self.contact_detector.contact,
            },
            sort_keys=True,
        )
        self.diagnostics_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PhysicalFtContactObserver()
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
