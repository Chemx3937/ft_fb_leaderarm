#!/usr/bin/env python3
"""Verify canonical ContactObservation use by IL collection and inference."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import rclpy
import yaml
from contact_observer_msgs.msg import ContactObservation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .contract import SAMPLE_HZ
from .feedback_authorization import file_sha256


SCHEMA_VERSION = 1
ANALYSIS_TYPE = "physical_ft_il_contact_contract_v1"
TOPIC = "/contact_observer/right/observation"
FRAME = "right_base_link"
MESSAGE_TYPE = "contact_observer_msgs/msg/ContactObservation"
MODE_CONSUMER = {
    "collection": "/chem_acp_raw_data_collection",
    "inference": "/chem_acp_env_runner",
}


def _endpoint_node(info):
    namespace = str(info.node_namespace).rstrip("/")
    return f"{namespace}/{info.node_name}" if namespace else f"/{info.node_name}"


def _config_failures(recorder, policy):
    failures = []
    if recorder.get("record_contact_observation") is not True:
        failures.append("recorder must enable record_contact_observation")
    if recorder.get("contact_observation_topic") != TOPIC:
        failures.append("recorder contact topic is not canonical")
    if recorder.get("observer_input_frame_id") != FRAME:
        failures.append("recorder contact frame is not right_base_link")
    try:
        recorder_hz = float(recorder.get("contact_observation_hz"))
    except (TypeError, ValueError):
        recorder_hz = math.nan
    if not math.isfinite(recorder_hz) or abs(recorder_hz - SAMPLE_HZ) > 1.0e-9:
        failures.append(f"recorder contact rate must be {SAMPLE_HZ} Hz")

    topics = policy.get("topics")
    preprocess = policy.get("preprocess")
    if not isinstance(topics, dict):
        topics = {}
    if not isinstance(preprocess, dict):
        preprocess = {}
    if topics.get("contact_observation") != TOPIC:
        failures.append("policy contact topic is not canonical")
    if topics.get("contact_observation_frame_id") != FRAME:
        failures.append("policy contact frame is not right_base_link")
    try:
        policy_hz = float(preprocess.get("contact_state_hz"))
    except (TypeError, ValueError):
        policy_hz = math.nan
    if not math.isfinite(policy_hz) or abs(policy_hz - SAMPLE_HZ) > 1.0e-9:
        failures.append(f"policy contact rate must be {SAMPLE_HZ} Hz")
    return failures


def analyze_il_contact_contract(
    recorder,
    policy,
    mode,
    publishers,
    subscribers,
    samples,
):
    if mode not in MODE_CONSUMER:
        raise RuntimeError(f"unknown IL verification mode: {mode}")
    failures = _config_failures(recorder, policy)
    typed_publishers = [
        row for row in publishers if row.get("topic_type") == MESSAGE_TYPE
    ]
    if len(publishers) != 1 or len(typed_publishers) != 1:
        failures.append("canonical contact topic must have exactly one publisher")
    required_consumer = MODE_CONSUMER[mode]
    matching_consumers = [
        row for row in subscribers
        if row.get("node") == required_consumer
        and row.get("topic_type") == MESSAGE_TYPE
    ]
    if len(matching_consumers) != 1:
        failures.append(
            f"required {mode} subscriber is missing or duplicated: "
            f"{required_consumer}"
        )

    malformed = 0
    ready_samples = 0
    states = set()
    for sample in samples:
        wrench = sample.get("contact_wrench")
        numeric_wrench = (
            isinstance(wrench, (list, tuple))
            and len(wrench) == 6
            and all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(float(value))
                for value in wrench
            )
        )
        state = sample.get("contact_state")
        valid = sample.get("valid") is True
        ready = sample.get("model_ready") is True
        well_formed = (
            sample.get("frame_id") == FRAME
            and state in (0, 1)
            and numeric_wrench
        )
        malformed += not well_formed
        if well_formed:
            states.add(int(state))
        ready_samples += well_formed and valid and ready
    if not samples:
        failures.append("no canonical ContactObservation messages were captured")
    if malformed:
        failures.append("malformed canonical ContactObservation messages exist")
    if not ready_samples:
        failures.append("no valid and model-ready ContactObservation was captured")

    return {
        "schema_version": SCHEMA_VERSION,
        "analysis_type": ANALYSIS_TYPE,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "passed": not failures,
        "mode": mode,
        "contract": {
            "topic": TOPIC,
            "frame_id": FRAME,
            "message_type": MESSAGE_TYPE,
            "sample_hz": SAMPLE_HZ,
            "required_consumer": required_consumer,
            "required_publishers": 1,
        },
        "graph": {
            "publishers": publishers,
            "subscribers": subscribers,
        },
        "metrics": {
            "captured_samples": len(samples),
            "valid_model_ready_samples": ready_samples,
            "malformed_samples": malformed,
            "observed_contact_states": sorted(states),
        },
        "failures": failures,
    }


class IlContactSampler(Node):
    def __init__(self):
        super().__init__("ft_il_contact_verifier")
        self.samples = []
        self.create_subscription(
            ContactObservation, TOPIC, self._callback, qos_profile_sensor_data
        )

    def _callback(self, message):
        self.samples.append(
            {
                "frame_id": str(message.header.frame_id),
                "contact_state": int(message.contact_state),
                "contact_wrench": list(message.contact_wrench),
                "valid": bool(message.valid),
                "model_ready": bool(message.model_ready),
            }
        )

    def graph(self):
        publishers = [
            {"node": _endpoint_node(info), "topic_type": str(info.topic_type)}
            for info in self.get_publishers_info_by_topic(TOPIC)
        ]
        subscribers = [
            {"node": _endpoint_node(info), "topic_type": str(info.topic_type)}
            for info in self.get_subscriptions_info_by_topic(TOPIC)
            if _endpoint_node(info) != "/ft_il_contact_verifier"
        ]
        return publishers, subscribers


def _load_yaml(path):
    target = Path(path).expanduser().resolve()
    if not target.is_file():
        raise RuntimeError(f"YAML file is missing: {target}")
    try:
        document = yaml.safe_load(target.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"failed to read YAML: {target}: {exc}") from exc
    if not isinstance(document, dict):
        raise RuntimeError(f"YAML root must be a mapping: {target}")
    return target, document


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Verify the canonical IL contact topic without commanding hardware"
    )
    parser.add_argument("--mode", choices=tuple(MODE_CONSUMER), required=True)
    parser.add_argument("--recorder-config", required=True)
    parser.add_argument("--policy-config", required=True)
    parser.add_argument("--duration-s", type=float, default=3.0)
    parser.add_argument("--output", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    node = None
    try:
        args = parse_args(argv)
        if not math.isfinite(args.duration_s) or args.duration_s <= 0.0:
            raise RuntimeError("duration-s must be finite and positive")
        output = Path(args.output).expanduser().resolve()
        if output.exists() or output.is_symlink():
            raise RuntimeError(f"refusing to overwrite IL contact report: {output}")
        recorder_path, recorder = _load_yaml(args.recorder_config)
        policy_path, policy = _load_yaml(args.policy_config)
        recorder_hash = file_sha256(recorder_path)
        policy_hash = file_sha256(policy_path)
        rclpy.init()
        node = IlContactSampler()
        deadline = time.monotonic() + args.duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        publishers, subscribers = node.graph()
        report = analyze_il_contact_contract(
            recorder,
            policy,
            args.mode,
            publishers,
            subscribers,
            node.samples,
        )
        if (
            file_sha256(recorder_path) != recorder_hash
            or file_sha256(policy_path) != policy_hash
        ):
            raise RuntimeError("IL config changed during verification")
        report["sources"] = {
            "recorder_config": str(recorder_path),
            "recorder_config_sha256": recorder_hash,
            "policy_config": str(policy_path),
            "policy_config_sha256": policy_hash,
            "requested_duration_s": args.duration_s,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(("PASS" if report["passed"] else "FAIL") + f": {output}")
        for failure in report["failures"]:
            print(f"- {failure}")
        return 0 if report["passed"] else 2
    except Exception as exc:
        print(f"ERROR: IL contact verification failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
