import csv
import json

import pytest

from ft_fb_leaderarm.feedback_analysis import (
    REQUIRED_COLUMNS,
    analyze_feedback_evidence,
)
from ft_fb_leaderarm.feedback_authorization import (
    ATTESTATIONS,
    create_feedback_authorization,
    file_sha256,
    validate_feedback_authorization,
)


def write_model(root):
    root.mkdir()
    model = root / "model.ts"
    model.write_bytes(b"approved synthetic model")
    (root / "metadata.json").write_text(
        '{"approved":true,"model_sha256":"' + file_sha256(model) + '"}'
    )
    return model


def write_csv(path, contact_pattern, feedback_on=False, gain_scale=0.0):
    fieldnames = sorted(REQUIRED_COLUMNS)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, contact in enumerate(contact_pattern):
            force = 2.5 if contact else 0.2
            row = {key: 0 for key in fieldnames}
            row.update(
                {
                    "t_s": 1.0 + 0.002 * index,
                    "dt_ms": 2.0,
                    "state": "FAST",
                    "feedback_gain_scale_contract": gain_scale,
                    "observer_valid": 1,
                    "observer_model_ready": 1,
                    "observer_source_age_ms": 1.0,
                    "observer_contact_state": int(contact),
                    "observer_contact_score_N": force,
                    "observer_prediction_age_ms": 1.0,
                    "observer_latency_ms": 1.0,
                    "observer_source_sequence": index + 1,
                    "observer_prediction_sequence": index + 1,
                    "fe_raw_fx": force,
                }
            )
            if contact and feedback_on:
                row["tau_fb_j1"] = 0.01
            writer.writerow(row)


def test_analyzer_counts_false_contact_and_stage_health(tmp_path):
    model = write_model(tmp_path / "model")
    free = []
    free_pattern = [False] * 5001
    for index in range(3):
        path = tmp_path / f"free_{index}.csv"
        write_csv(path, free_pattern)
        free.append(path)
    contact = tmp_path / "contact.csv"
    contact_pattern = [False] * 5001
    for index in (500, 1500, 2500):
        contact_pattern[index] = True
    write_csv(contact, contact_pattern)
    limits = {
        "max_contact_force_n": 5.0,
        "max_free_force_error_n": 1.0,
        "max_pose_step_deg": 1.0,
        "max_velocity_reversal_hz": 8.0,
        "max_source_age_ms": 20.0,
        "max_record_gap_ms": 10.0,
        "min_run_duration_s": 10.0,
        "min_csv_hz": 250.0,
        "min_contact_activations": 3,
        "min_contact_feedback_nonzero_fraction": 0.95,
    }
    report = analyze_feedback_evidence(model, 0.40, free, [contact], limits)
    assert report["passed"]
    assert report["aggregate"]["false_contact_activations"] == 0
    assert report["aggregate"]["controlled_contact_activations"] == 3

    bad_free_pattern = list(free_pattern)
    bad_free_pattern[500] = True
    write_csv(free[0], bad_free_pattern)
    rejected = analyze_feedback_evidence(model, 0.40, free, [contact], limits)
    assert not rejected["passed"]
    assert "FREE runs contain false CONTACT activations" in rejected["failures"]


def test_authorization_recomputes_bound_off_and_40_percent_csv(tmp_path):
    model = write_model(tmp_path / "model")
    limits = {
        "max_contact_force_n": 5.0,
        "max_free_force_error_n": 1.0,
        "max_pose_step_deg": 1.0,
        "max_velocity_reversal_hz": 8.0,
        "max_source_age_ms": 20.0,
        "max_record_gap_ms": 10.0,
        "min_run_duration_s": 10.0,
        "min_csv_hz": 250.0,
        "min_contact_activations": 3,
        "min_contact_feedback_nonzero_fraction": 0.95,
    }
    free_pattern = [False] * 5001
    contact_pattern = [False] * 5001
    for index in (500, 1500, 2500):
        contact_pattern[index] = True

    def make_report(prefix, target, feedback_on):
        free = []
        for index in range(3):
            path = tmp_path / f"{prefix}_free_{index}.csv"
            write_csv(
                path,
                free_pattern,
                feedback_on,
                0.40 if feedback_on else 0.0,
            )
            free.append(path)
        contact = tmp_path / f"{prefix}_contact.csv"
        write_csv(
            contact,
            contact_pattern,
            feedback_on,
            0.40 if feedback_on else 0.0,
        )
        report = analyze_feedback_evidence(model, target, free, [contact], limits)
        assert report["passed"]
        report_path = tmp_path / f"{prefix}_analysis.json"
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        return report_path, contact

    off_report, _ = make_report("off", 0.40, False)
    stage40 = create_feedback_authorization(
        tmp_path / "feedback_40.json",
        model,
        0.40,
        [off_report],
        ATTESTATIONS[1],
    )
    gain40_report, gain40_contact = make_report("gain40", 1.00, True)
    stage100 = create_feedback_authorization(
        tmp_path / "feedback_100.json",
        model,
        1.00,
        [gain40_report],
        ATTESTATIONS[2],
        stage40,
    )
    assert validate_feedback_authorization(stage100, model, 1.00)["stage"] == 2
    gain40_contact.write_text("changed after authorization\n")
    with pytest.raises(RuntimeError, match="raw CSV changed"):
        validate_feedback_authorization(stage100, model, 1.00)
