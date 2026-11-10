"""Diagnostic three-model inference; never publishes robot control messages."""

from collections import deque
import json
import math
import csv
from pathlib import Path

import numpy as np

from .contract import CausalFeatureBuilder, SAMPLE_HZ, SchmittContactDetector


ROBOT_TOPIC = "/bae_r/observer_input"
WRIST_TOPIC = "/aft_sensor2/wrench"
REFERENCE_TOPIC = "/aft_sensor1/wrench"
FORCE_BOUNDARY_N = 3.0
TRIAL_TOPICS = {
    "free": (ROBOT_TOPIC, WRIST_TOPIC),
    "weak": (ROBOT_TOPIC, WRIST_TOPIC, REFERENCE_TOPIC),
    "normal": (ROBOT_TOPIC, WRIST_TOPIC, REFERENCE_TOPIC),
    "ramp": (ROBOT_TOPIC, WRIST_TOPIC, REFERENCE_TOPIC),
    "noise": (ROBOT_TOPIC, WRIST_TOPIC, REFERENCE_TOPIC),
}


def force_band(value):
    if value is None or not math.isfinite(float(value)):
        return "UNAVAILABLE"
    return "≤ 3 N" if float(value) <= FORCE_BOUNDARY_N else "> 3 N"


def averaged_reference_force(rows, baseline, window_s):
    """Causal display aid only; inference, reference labels and scoring stay raw."""
    if not rows or baseline is None:
        return None
    end = rows[-1]["t"]
    # Timestamp floats can round an exact window endpoint just outside the bound.
    selected = [r for r in rows if 0 <= end-r["t"] <= window_s+1e-6]
    if len(selected) < 3 or selected[-1]["t"]-selected[0]["t"] < .8*window_s:
        return None
    if any(r.get("reference_force_n") is None or r.get("reference_wrench") is None for r in selected):
        return None
    if np.max(np.diff([r["t"] for r in selected])) > .020:
        return None
    forces = np.array([r["reference_wrench"][:3] for r in selected])-baseline
    return float(np.linalg.norm(forces.mean(axis=0))) if np.isfinite(forces).all() else None


def external_force_bins(forces, states, common_valid):
    """CONTACT output fraction by unfiltered external force, not recall/accuracy."""
    forces, states = np.asarray(forces), np.asarray(states)
    valid = np.asarray(common_valid, dtype=bool) & np.isfinite(forces)
    masks = {"below_2p5": forces < 2.5, "2p5_to_3": (forces >= 2.5) & (forces <= 3.),
             "3_to_3p5": (forces > 3.) & (forces <= 3.5), "above_3p5": forces > 3.5}
    result = {}
    for name, band in masks.items():
        mask = valid & band; count = int(mask.sum())
        contact = int(np.sum(mask & (states == "CONTACT")))
        result[name] = {"samples": count, "duration_s": count/SAMPLE_HZ,
                        "mean_external_force_n": float(forces[mask].mean()) if count else None,
                        "contact_samples": contact, "contact_fraction": contact/count if count else None}
    return result


def noise_summary(forces):
    values = np.asarray(forces, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("noise needs finite [N,3] stationary force samples")
    center = np.median(values, axis=0)
    centered = np.linalg.norm(values - center, axis=1)
    return {
        "baseline_n": center.tolist(),
        "baseline_norm_n": float(np.linalg.norm(center)),
        "axis_std_n": np.std(values, axis=0).tolist(),
        "centered_norm_p99_n": float(np.percentile(centered, 99)),
        "centered_norm_max_n": float(centered.max()),
        "meaning": "stationary noise and zero offset; not absolute sensor accuracy",
    }


def load_comparison(session_path):
    """Keep diagnostic models sealed and check the shared training conditions."""
    from .model import BundlePredictor

    path = Path(session_path).expanduser().resolve()
    manifest = json.loads(path.read_text())
    from .comparison_trace import LIVE_RAMP_PROTOCOL, DETECTOR
    if manifest.get('ramp_trial',{}).get('mode') == LIVE_RAMP_PROTOCOL and manifest.get('observer_detector') != DETECTOR:
        raise ValueError('live ramp requires the fixed3.0/1.2N20/20ms observer configuration')
    root = Path(manifest["package_root"])
    models = {}
    if len(manifest["models"]) != 3:
        raise ValueError("comparison requires exactly three fixed models")
    for spec in manifest["models"]:
        predictor = BundlePredictor(root / spec["bundle"], require_approved=False)
        if (predictor.model_sha256 != spec["model_sha256"] or
                predictor.metadata_sha256 != spec["metadata_sha256"]):
            raise ValueError("comparison artifact differs from the selected SHA pair")
        for key in ("payload_id", "controller_config_hash"):
            if predictor.metadata.get(key) != manifest[key]:
                raise ValueError(f"model {key} differs from session")
        if predictor.metadata.get("observer_input_frame") != "right_base_link" or predictor.metadata.get("ft_frame") != "aft_sensor2":
            raise ValueError("comparison model frames differ from live sensor frames")
        models[spec["name"]] = predictor
    noise = None
    diagnostic = manifest.get("post_zero_static_diagnostic", {})
    report_path = path.parent / diagnostic.get("report", "missing")
    if report_path.is_file():
        report = json.loads(report_path.read_text())
        if report.get("zero_set_id") == manifest["zero_set_id"] and report.get("static_zero_gate_pass"):
            noise = report
    return manifest, ComparisonEngine(models, noise), noise


class ComparisonEngine:
    """Use one causal input history and native per-bundle feature projection."""

    def __init__(self, predictors, noise=None):
        if not predictors:
            raise ValueError("no predictors")
        self.predictors = predictors
        self.names = list(predictors)
        self.history = deque(maxlen=max(p.history for p in predictors.values()))
        self.builder = CausalFeatureBuilder()
        self.detectors = {k: SchmittContactDetector(3.0, 1.2, .020, .020) for k in predictors}
        self.last_sequence = None
        self.frames = deque(maxlen=5500)
        self.noise = noise
        self.reference_baseline = None if noise is None else np.asarray(noise["sensors"]["1"]["force_median_n"])
        if self.reference_baseline is not None and (self.reference_baseline.shape != (3,) or not np.isfinite(self.reference_baseline).all()):
            raise ValueError("external baseline must have three finite values")

    def clear(self):
        self.history.clear()
        self.builder.reset()
        for detector in self.detectors.values():
            detector.reset()

    @staticmethod
    def fresh(sample, now, frame):
        if sample is None or sample.get("frame") != frame or not sample.get("valid", True):
            return False
        try:
            age, local_age = now - sample["stamp"], now - sample["received"]
            return -.002 <= age <= .020 and 0 <= local_age <= .020
        except (KeyError, TypeError):
            return False

    def step(self, now, state, wrist, reference=None):
        row = {"t": float(now), "measured": None, "predictions": {}, "states": {}, "residual_norms": {}, "reference_force_n": None, "reference_band": "UNAVAILABLE",
               "input_status":"VALID", "model_ready":dict.fromkeys(self.names,False)}
        wrist_ok = self.fresh(wrist, now, "aft_sensor2")
        if wrist_ok:
            row["measured"] = np.asarray(wrist["wrench"], dtype=float).copy()
            wrist_ok = row["measured"].shape == (6,) and np.isfinite(row["measured"]).all()
        if wrist_ok:
            row["wrist_stamp"] = wrist["stamp"]
        else:
            row["measured"] = None
        ref_ok = self.fresh(reference, now, "aft_sensor1")
        if ref_ok and self.reference_baseline is not None:
            ref = np.asarray(reference["wrench"], dtype=float)
            if ref.shape == (6,) and np.isfinite(ref).all():
                row["reference_force_n"] = float(np.linalg.norm(ref[:3] - self.reference_baseline))
                row["reference_band"] = force_band(row["reference_force_n"])
                row["reference_stamp"] = reference["stamp"]
                row["reference_wrench"] = ref.copy()
        state_ok = self.fresh(state, now, "right_base_link")
        reason = "VALID"
        if not wrist_ok or not state_ok:
            reason = "STALE / INVALID"
        elif abs(state["stamp"] - wrist["stamp"]) > .003:
            reason = "UNSYNCHRONIZED"
        elif state["sequence"] == self.last_sequence:
            reason = "DUPLICATE"
        if reason != "VALID":
            row['input_status'] = reason
            if reason != "DUPLICATE":
                self.clear()
            row["states"] = dict.fromkeys(self.names, reason)
            self.frames.append(row)
            return row
        feature = self.builder.build(state["q"], state["dq"], state["stamp"])
        if feature is None:
            row['input_status'] = 'INVALID FEATURES'
            self.clear()
            row["states"] = dict.fromkeys(self.names, "INVALID FEATURES")
        else:
            self.last_sequence = state["sequence"]
            self.history.append(feature.astype(np.float32))
            for name, predictor in self.predictors.items():
                if len(self.history) < predictor.history:
                    row["states"][name] = "WARMUP"
                    continue
                row['model_ready'][name] = True
                try:
                    prediction = predictor.predict(np.stack(list(self.history)[-predictor.history:]))
                    if np.asarray(prediction).shape != (6,) or not np.isfinite(prediction).all():
                        raise ValueError("invalid prediction")
                    norm = float(np.linalg.norm(row["measured"][:3] - prediction[:3]))
                    row["predictions"][name] = prediction
                    row["residual_norms"][name] = norm
                    row["states"][name] = "CONTACT" if self.detectors[name].update(norm, now) else "FREE"
                except Exception as exc:
                    self.detectors[name].reset()
                    row["states"][name] = "ERROR: " + str(exc)[:120]
        self.frames.append(row)
        return row


class ExternalLoadReference:
    """Independent load reference shared by guidance and offline scoring."""

    def __init__(self, protocol):
        self.protocol = {k: float(protocol.get(k, default)) for k, default in
                         (("force_on_n", 1.6), ("force_off_n", 1.),
                          ("contact_hold_ms", 20.), ("free_hold_ms", 20.))}
        if not all(math.isfinite(v) for v in self.protocol.values()) or min(self.protocol["contact_hold_ms"], self.protocol["free_hold_ms"]) < 0:
            raise ValueError("invalid external load reference protocol")
        self.detector = SchmittContactDetector(self.protocol["force_on_n"], self.protocol["force_off_n"],
                                              self.protocol["contact_hold_ms"]/1000, self.protocol["free_hold_ms"]/1000)
        self.reset()

    def reset(self, stamp=None):
        self.detector.reset()
        self.seen_s = stamp
        self.state = self.low_since = None

    def invalidate(self):
        self.detector.pending_since_s = None
        self.state = self.low_since = None

    def update(self, stamp, force):
        gap = self.seen_s is not None and (stamp <= self.seen_s or stamp-self.seen_s > .020)
        self.seen_s = stamp
        if gap or force is None or not math.isfinite(force):
            self.invalidate()
            return None
        contact = self.detector.update(force, stamp)
        if self.state is not None:
            self.state = bool(contact); self.low_since = None
        elif force <= self.protocol["force_off_n"]:
            if self.low_since is None: self.low_since = stamp
            if not contact and stamp-self.low_since >= self.protocol["free_hold_ms"]/1000:
                self.state = False
        elif contact and force >= self.protocol["force_on_n"]:
            self.state = True; self.low_since = None
        else:
            self.low_since = None
        return self.state


def external_reference_states(rows, protocol):
    reference = ExternalLoadReference(protocol)
    return np.array([-1 if (state := reference.update(r["t"], r.get("reference_force_n"))) is None else int(state)
                     for r in rows], dtype=np.int8)


def paired_reference(row):
    force = row.get("reference_force_n")
    return (force is not None and math.isfinite(force) and
            abs(row.get("reference_stamp", -1)-row.get("wrist_stamp", -10)) <= .003)


def reference_events(rows, labels):
    """Unknown samples split intervals and never manufacture a release."""
    events = []
    for i, state in enumerate(labels):
        if state == 1 and (i == 0 or labels[i-1] != 1):
            events.append({"start_index": i, "stop_index": len(rows), "start_s": rows[i]["t"], "end_s": None,
                           "left_censored": bool(i == 0 or labels[i-1] != 0), "right_censored": True})
        elif state != 1 and i and labels[i-1] == 1:
            events[-1].update(stop_index=i, end_s=rows[i]["t"] if state == 0 else None,
                              right_censored=bool(state != 0))
    return events


def contact_metrics(rows, names, labels, common_valid):
    """Descriptive metrics against the load reference, with no acceptance gate."""
    labels = np.asarray(labels)
    paired = np.array([paired_reference(r) for r in rows])
    valid = np.asarray(common_valid) & paired & (labels >= 0)
    events = reference_events(rows, labels)
    result = {"definition": "external-load-reference-relative; not exact physical first-touch ground truth",
              "scored_samples": int(valid.sum()), "unknown_reference_samples": int((labels < 0).sum()),
              "common_invalid_or_warmup_samples": int((~np.asarray(common_valid, dtype=bool)).sum()),
              "unpaired_reference_samples": int((~paired).sum()),
              "reference_events": events, "models": {}}
    for name in names:
        states = np.array([r["states"].get(name, "INVALID") for r in rows])
        contact = states == "CONTACT"
        edges = np.r_[False, (states[:-1] == "FREE") & contact[1:]]
        falls = np.r_[False, contact[:-1] & (states[1:] == "FREE")]
        continuous = np.r_[False, valid[:-1] & valid[1:] & (np.diff([r["t"] for r in rows]) <= .020)]
        tp = int(np.sum(valid & contact & (labels == 1)))
        fp = int(np.sum(valid & contact & (labels == 0)))
        fn = int(np.sum(valid & ~contact & (labels == 1)))
        tn = int(np.sum(valid & ~contact & (labels == 0)))
        model_events = []
        for j, event in enumerate(events):
            a, b = event["start_index"], event["stop_index"]
            next_start = events[j+1]["start_index"] if j+1 < len(events) else len(rows)
            preactive = a > 0 and contact[a-1] and contact[a] and valid[a-1] and valid[a]
            candidates = np.flatnonzero(edges[a:b] & continuous[a:b]) + a
            onset = int(candidates[0]) if len(candidates) and not preactive and not event["left_censored"] else None
            active = np.flatnonzero(contact[a:b] & valid[a:b]) + a
            release = None
            if len(active) and event["end_s"] is not None:
                last = int(active[-1])
                # Missing/invalid input between CONTACT and FREE is an unknown release.
                for k in range(last+1, next_start):
                    if not continuous[k]: break
                    if falls[k]:
                        release = k; break
            model_events.append({**event, "preactivated_at_reference_onset": bool(preactive),
                                 "contact_observed": bool(len(active)), "new_contact_edges": int(len(candidates)),
                                 "onset_latency_ms": None if onset is None else 1000*(rows[onset]["t"]-event["start_s"]),
                                 "release_latency_ms": None if release is None else 1000*(rows[release]["t"]-event["end_s"])})
        entry = {"true_positive_samples": tp, "false_positive_samples": fp,
                 "false_negative_samples": fn, "true_negative_samples": tn,
                 "precision": tp/(tp+fp) if tp+fp else None, "recall": tp/(tp+fn) if tp+fn else None,
                 "false_contact_time_s": fp/SAMPLE_HZ,
                 "pre_reference_contact_edges": int(np.sum(edges & continuous & (labels == 0))),
                 "reference_contact_events": len(events),
                 "events_with_contact_output": sum(e["contact_observed"] for e in model_events),
                 "events_with_new_onset": sum(e["onset_latency_ms"] is not None for e in model_events),
                 "preactivated_events": sum(e["preactivated_at_reference_onset"] for e in model_events),
                 "events": model_events}
        for key in ("onset_latency_ms", "release_latency_ms"):
            values = [event[key] for event in model_events if event[key] is not None]
            entry[key+"_median"] = float(np.median(values)) if values else None
            entry[key+"_max"] = max(values) if values else None
        result["models"][name] = entry
    return result


def ramp_onsets(rows, names, reference_states=None, upper_n=None):
    """New CONTACT edges during the first reference contact's loading phase."""
    labels = external_reference_states(rows, {}) if reference_states is None else reference_states
    events = reference_events(rows, labels)
    result = dict.fromkeys(names)
    if not events or events[0]["left_censored"]: return result
    a, b = events[0]["start_index"], events[0]["stop_index"]
    if upper_n is not None:
        b = next((i+1 for i in range(a,b) if rows[i].get("reference_force_n") is not None and
                  rows[i]["reference_force_n"] >= upper_n), b)
    for name in names:
        if rows[a-1]["states"].get(name) == "CONTACT": continue
        for i in range(a,b):
            row, previous = rows[i], rows[i-1]
            if row["t"]-previous["t"] > .020 or any(r["states"].get(n) not in ("FREE", "CONTACT") for r in (previous,row) for n in names): continue
            if previous["states"].get(name) != "FREE" or row["states"].get(name) != "CONTACT": continue
            paired = paired_reference(row)
            window = [r for r in rows[a:i+1] if row["t"]-r["t"] <= .25 and r.get("reference_force_n") is not None]
            rate = None
            if len(window) >= 3 and window[-1]["t"]-window[0]["t"] >= .1:
                rate = float(np.polyfit([r["t"]-row["t"] for r in window], [r["reference_force_n"] for r in window], 1)[0])
            result[name] = {"t_s": row["t"], "external_force_n": row["reference_force_n"] if paired else None,
                            "residual_force_n": row["residual_norms"][name], "reference_valid": bool(paired),
                            "local_force_rise_rate_n_s": rate}
            break
    return result


def replay_frames(engine, bag_path):
    """Same engine and receive-time pairing, without connecting to live ROS."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from .contract import stamp_to_seconds, wrench_from_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(Path(bag_path).resolve()), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    samples, ticks, origin, tick_index, latest = [], None, None, 0, {}
    while reader.has_next():
        topic, blob, received = reader.read_next()
        now = received * 1e-9
        if ticks is None:
            origin = ticks = now
        while ticks < now:
            samples.append(engine.step(ticks, latest.get(ROBOT_TOPIC), latest.get(WRIST_TOPIC), latest.get(REFERENCE_TOPIC)))
            tick_index += 1
            ticks = origin + tick_index / SAMPLE_HZ
        if topic not in TRIAL_TOPICS["noise"]:
            continue
        msg = deserialize_message(blob, types[topic])
        sample = {"stamp": stamp_to_seconds(msg.header.stamp), "received": now, "frame": msg.header.frame_id}
        if topic == ROBOT_TOPIC:
            sample.update(q=np.array(msg.q_rad), dq=np.array(msg.dq_rad_s), sequence=int(msg.source_sequence), valid=bool(msg.valid))
        else:
            sample["wrench"] = wrench_from_message(msg)
        latest[topic] = sample
    return samples


def analyze_trial(directory, session_path):
    """Recompute all models from raw bag, never from decimated screen curves."""
    from .contract import error_metrics
    directory = Path(directory)
    metadata = json.loads((directory / "trial.json").read_text())
    from .comparison_trace import LIVE_RAMP_PROTOCOL, analyze_live_ramp
    if metadata['kind'] == 'ramp':
        manifest = json.loads(Path(session_path).read_text())
        if manifest.get('ramp_trial', {}).get('mode') == LIVE_RAMP_PROTOCOL:
            if metadata['zero_set_id'] != manifest['zero_set_id']:
                raise ValueError('live trial zero-set differs from session')
            return analyze_live_ramp(directory, manifest)
    manifest, engine, _ = load_comparison(session_path)
    if metadata["zero_set_id"] != manifest["zero_set_id"]:
        raise ValueError("use the session manifest from this trial's zero-set")
    rows = replay_frames(engine, directory / "raw")
    begin = metadata["score_start_s"]
    end = metadata["score_end_s"]
    scored = [r for r in rows if begin <= r["t"] < end]
    if not scored:
        raise ValueError("no recorded data in the scoring interval")
    result = {"analysis_schema": "three_model_comparison_v2", "kind": metadata["kind"], "owner": metadata["owner"], "zero_set_id": manifest["zero_set_id"],
              "source": "offline raw bag replay, common 262.5 Hz ticks and native model preprocessing",
              "detector": {"on_n": 3.0, "off_n": 1.2, "contact_hold_ms": 20, "free_hold_ms": 20},
              "models": {}, "ramp_onsets": None}
    if metadata["kind"] != "free":
        result["external_contact_reference_protocol"] = manifest.get("external_contact_reference_protocol")
    arrays = {"t_s": np.array([r["t"] for r in scored]),
              "measured": np.array([r["measured"] if r["measured"] is not None else np.full(6, np.nan) for r in scored])}
    common_valid = np.array([all(r["states"].get(n) in ("FREE", "CONTACT") for n in engine.names) for r in scored])
    result["common_valid_samples"] = int(common_valid.sum())
    if not common_valid.any():
        raise ValueError("no shared valid model predictions in the scoring interval")
    if metadata["kind"] != "free":
        arrays["external_force_n"] = np.array([r["reference_force_n"] if r["reference_force_n"] is not None else np.nan for r in scored])
        labels = external_reference_states(rows, manifest.get("external_contact_reference_protocol", {}))
        arrays["external_reference_state"] = labels[np.array([begin <= r["t"] < end for r in rows])]
        for key in ("reference_stamp", "wrist_stamp"):
            arrays[key] = np.array([r.get(key, np.nan) for r in scored])
        evaluation = contact_metrics(scored, engine.names, arrays["external_reference_state"], common_valid)
        result["contact_evaluation"] = evaluation
        events = evaluation["reference_events"]
        forces = arrays["external_force_n"]; finite = forces[np.isfinite(forces)]
        result["protocol_quality"] = {
            "reference_contact_intervals": len(events),
            "single_complete_reference_contact": len(events) == 1 and not events[0]["left_censored"] and not events[0]["right_censored"],
            "external_force_max_n": float(finite.max()) if len(finite) else None,
            "weak_exceeded_normal_target": bool(metadata["kind"] == "weak" and len(finite) and finite.max() > manifest.get("normal_contact_target_n", 5.)),
            "meaning": "external-load protocol selection only; preserve raw data and never select trials according to model CONTACT output"}
        above_since = None
        sustained_strong = False
        for row in scored:
            if row.get("reference_force_n") is not None and row["reference_force_n"] > manifest.get("normal_contact_target_n", 5.):
                if above_since is None: above_since = row["t"]
                sustained_strong |= row["t"]-above_since >= .020
            else: above_since = None
        quality = result["protocol_quality"]
        quality["weak_sustained_above_normal_target"] = bool(metadata["kind"] == "weak" and sustained_strong)
        quality["plan_eligible"] = bool(quality["single_complete_reference_contact"] and not quality["weak_sustained_above_normal_target"])
        quality["selection_basis"] = "single complete external contact; weak trial must not exceed normal5N target for20ms; independent of observer CONTACT outputs"
        release = next((event["t_s"] for event in metadata.get("guide_events", []) if event["stage"] == "release"), None)
        if release is not None and metadata["kind"] in ("weak", "normal"):
            hold = common_valid & (arrays["t_s"] >= release-2) & (arrays["t_s"] < release)
            result["guided_hold_window"] = {
                "definition": "last 2s before GUI release cue; operator guidance window, not an independent contact label",
                "samples": int(hold.sum()),
                "mean_external_force_n": float(np.nanmean(forces[hold])) if hold.any() and np.isfinite(forces[hold]).any() else None,
                "models": {name: {"contact_samples": sum(r["states"][name] == "CONTACT" for r, keep in zip(scored, hold) if keep),
                                  "contact_fraction": sum(r["states"][name] == "CONTACT" for r, keep in zip(scored, hold) if keep)/int(hold.sum()) if hold.any() else None}
                           for name in engine.names}}
        if metadata["kind"] == "ramp":
            result["ramp_onsets"] = ramp_onsets(scored, engine.names, arrays["external_reference_state"], manifest.get("ramp_trial", {}).get("force_upper_n"))
            result["ramp_onset_scope"] = "new FREE-to-CONTACT edge within first independently referenced contact, no later than first upper-force crossing; preactivation is not an onset"
    for name in engine.names:
        prediction = np.array([r["predictions"].get(name, np.full(6, np.nan)) for r in scored])
        states = np.array([r["states"].get(name, "INVALID") for r in scored])
        mask = np.isin(states, ["FREE", "CONTACT"])
        contact = states == "CONTACT"
        entry = {"valid_samples": int(mask.sum()), "invalid_or_warmup_samples": int((~mask).sum()),
                 "contact_events": int(np.count_nonzero(contact & ~np.r_[False, contact[:-1]])),
                 "contact_time_s": float(contact.sum() / SAMPLE_HZ),
                 "model_sha256": engine.predictors[name].model_sha256,
                 "metadata_sha256": engine.predictors[name].metadata_sha256}
        if metadata["kind"] == "free" and common_valid.any():
            entry["free_force_error"] = error_metrics(arrays["measured"][common_valid], prediction[common_valid])
            entry["false_contact_events"] = entry["contact_events"]
            entry["false_contact_time_s"] = entry["contact_time_s"]
            entry["segments"] = {}
            for segment, low, high in [("start_stationary",0,10),("slow",10,30),("normal_speed",30,50),("end_stationary",50,60)]:
                segment_mask = common_valid & (arrays["t_s"] >= begin+low) & (arrays["t_s"] < begin+high)
                if segment_mask.any():
                    entry["segments"][segment] = error_metrics(arrays["measured"][segment_mask], prediction[segment_mask])
        if metadata["kind"] != "free":
            entry["external_force_bins"] = external_force_bins(arrays["external_force_n"], states, common_valid)
        result["models"][name] = entry
        arrays[name + "_prediction"] = prediction
        arrays[name + "_state"] = states
    result["limitations"] = ["Contact residual is not free-space model error during physical contact.",
                              "Ramp force is force at a reference-qualified FREE-to-CONTACT decision, including 20ms hold; not a changed detector threshold or exact minimum detectable force.",
                              "A missing or unpaired ramp onset is reported as missing, never as zero force.",
                              "Force bins use raw external force at common replay ticks; CONTACT fractions are not accuracy or recall and may include actual FREE samples.",
                              "Contact reference uses a nonzero external load threshold; exact physical first-touch timing is not independently annotated.",
                              "External load reference and model states are sampled on common 262.5Hz replay ticks; sensor header times gate freshness/pairing. Reference hold and sample quantization affect reported latency.",
                              "Unknown/unpaired reference samples are excluded and counted; precision/recall are relative to the frozen external load protocol, not model-accuracy metrics or a runtime acceptance gate.",
                              "Multiple contact intervals or strong overshoot in a weak trial require protocol review; whole-file CONTACT presence does not prove weak-contact detection.",
                              "External sensor noise and a single known-mass check do not establish absolute force calibration accuracy.",
                              "Offline replay decision timing does not include live inference computation or controller end-to-end delay."]
    np.savez_compressed(directory / "comparison.npz", **arrays)
    (directory / "comparison_report.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    metadata["analysis_complete"] = True
    (directory / "trial.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n")
    if metadata["kind"] == "ramp":
        summarize_ramps(directory.parents[2])
    return result


def summarize_ramps(root):
    root = Path(root)
    rows, forces, trials, excluded = [], {}, 0, []
    for path in sorted(root.glob("session_*/trials/*/comparison_report.json")):
        report = json.loads(path.read_text())
        meta = json.loads((path.parent / "trial.json").read_text())
        if report["kind"] != "ramp" or not meta.get("complete"):
            continue
        if report.get("protocol_quality", {}).get("plan_eligible") is False:
            excluded.append(str(path.parent.relative_to(root)))
            continue
        trials += 1
        for name, onset in report["ramp_onsets"].items():
            forces.setdefault(name, [])
            value = onset["external_force_n"] if onset and onset["reference_valid"] else None
            if value is not None: forces[name].append(value)
            rows.append({"trial": str(path.parent.relative_to(root)), "model": name,
                         "decision_t_s": onset["t_s"] if onset else None,
                         "external_force_n": value,
                         "reference_valid": bool(onset and onset["reference_valid"])})
    summary = {"physical_trials": trials, "planned_shared_trials": 10,
               "excluded_protocol_trials": excluded, "models": {}}
    for name, values in forces.items():
        summary["models"][name] = {"paired_detections": len(values), "missing_or_unpaired": trials-len(values),
                                   "force_min_n": min(values) if values else None,
                                   "force_median_n": float(np.median(values)) if values else None,
                                   "force_max_n": max(values) if values else None,
                                   "force_std_n": float(np.std(values)) if values else None}
    (root / "ramp_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    with (root / "ramp_onsets.csv").open("w", newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=["trial","model","decision_t_s","external_force_n","reference_valid"])
        writer.writeheader();writer.writerows(rows)
    return summary


def analyze_noise(directory, session_path):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from .contract import FixedPoseZeroVerifier, stamp_to_seconds, wrench_from_message

    directory, session_path = Path(directory), Path(session_path)
    meta = json.loads((directory / "trial.json").read_text())
    manifest = json.loads(session_path.read_text())
    if meta["zero_set_id"] != manifest["zero_set_id"]:
        raise ValueError("noise trial zero-set differs from session")
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory / "raw"), storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    verifier, values = FixedPoseZeroVerifier(), {"1": [], "2": []}
    state = None
    gate_results = []
    start, end = meta["score_start_s"], meta["score_end_s"]
    while reader.has_next():
        topic, blob, received = reader.read_next()
        now = received * 1e-9
        msg = deserialize_message(blob, types[topic])
        stamp = stamp_to_seconds(msg.header.stamp)
        if topic == ROBOT_TOPIC:
            state = (msg, now, stamp)
        elif start <= now < end:
            num = "1" if topic == REFERENCE_TOPIC else "2"
            if msg.header.frame_id != f"aft_sensor{num}":
                raise ValueError("noise sensor frame mismatch")
            wrench = wrench_from_message(msg)
            values[num].append(wrench[:3])
            if num == "2":
                if state is None or not state[0].valid or state[0].header.frame_id != "right_base_link" or abs(stamp - state[2]) > .003 or now - state[1] > .020:
                    verifier.reset("invalid_or_unsynchronized_stream")
                    ready = False
                else:
                    ready = verifier.update(now, state[0].q_rad, state[0].dq_rad_s, wrench)
                if now - start >= 1.05:
                    gate_results.append(ready)
    if end - start < 9.9 or any(len(v) < 2000 for v in values.values()):
        raise ValueError("noise record needs a complete stationary 10s interval")
    report = {"zero_set_id": meta["zero_set_id"], "sensors": {},
              "static_zero_gate_pass": bool(gate_results and all(gate_results) and verifier.status()[0])}
    for num, v in values.items():
        summary = noise_summary(v)
        report["sensors"][num] = {"force_median_n": summary["baseline_n"], "force_median_norm_n": summary["baseline_norm_n"],
                                  "force_axis_std_n": summary["axis_std_n"],
                                  "centered_force_norm_p99_p99_9_max_n": [summary["centered_norm_p99_n"], float(np.percentile(np.linalg.norm(np.asarray(v) - summary["baseline_n"], axis=1), 99.9)), summary["centered_norm_max_n"]],
                                  "samples": len(v)}
    report["meaning"] = "stationary sensor noise and bias; not absolute calibration accuracy"
    report_path = directory / "noise_report.json"
    report_path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    meta["analysis_complete"] = True
    (directory / "trial.json").write_text(json.dumps(meta, indent=2, ensure_ascii=False) + "\n")
    if report["static_zero_gate_pass"]:
        manifest["post_zero_static_diagnostic"] = {"report": str(report_path.resolve()), "result": "passed", "zero_set_id": meta["zero_set_id"]}
        temporary = session_path.with_suffix(".tmp.json")
        temporary.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
        temporary.replace(session_path)
    return report
