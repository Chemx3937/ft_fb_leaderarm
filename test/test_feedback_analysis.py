import csv
import json

import pytest

from ft_fb_leaderarm.contract import APPROVAL_CONTRACT
from ft_fb_leaderarm.feedback_analysis import (
    REQUIRED_COLUMNS,
    SAFETY_LIMITS,
    analyze_feedback_evidence,
    analyze_feedback_onsets,
    onset_main,
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
        json.dumps(
            {
                "approved": True,
                "approval_contract": APPROVAL_CONTRACT,
                "model_sha256": file_sha256(model),
            }
        )
    )
    return model


def write_csv(
    path, contact_pattern, feedback_on=False, gain_scale=0.0, free_force=0.2
):
    fieldnames = sorted(REQUIRED_COLUMNS)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, contact in enumerate(contact_pattern):
            force = 2.5 if contact else free_force
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


def write_onset_csv(path):
    fieldnames = (
        "t_s",
        "state",
        "feedback_gain_scale_contract",
        "observer_valid",
        "observer_model_ready",
        "fe_age_ms",
        "observer_source_age_ms",
        "observer_contact_state",
        "contact_scale",
        *(f"tau_fb_j{index}" for index in range(1, 7)),
    )
    rows = []
    for _ in range(3):
        rows.append((0, 0.0))
        rows.extend((1, scale) for scale in (0.0, 0.25, 0.5, 0.75, 1.0, 1.0))
        rows.append((0, 0.0))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for index, (contact, scale) in enumerate(rows):
            row = {key: 0 for key in fieldnames}
            row.update(
                {
                    "t_s": index * 0.002,
                    "state": "FAST",
                    "feedback_gain_scale_contract": 0.4,
                    "observer_valid": 1,
                    "observer_model_ready": 1,
                    "fe_age_ms": 1.0,
                    "observer_source_age_ms": 1.0,
                    "observer_contact_state": contact,
                    "contact_scale": scale,
                    "tau_fb_j1": 0.04 * scale,
                }
            )
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
    limits = {**SAFETY_LIMITS, "max_contact_force_n": 5.0}
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

    write_csv(free[0], free_pattern, free_force=1.6)
    rejected = analyze_feedback_evidence(model, 0.40, free, [contact], limits)
    assert not rejected["passed"]
    assert (
        "FREE residual force p99 exceeded the operational limit"
        in rejected["failures"]
    )


def test_onset_analyzer_checks_ramp_step_and_blocked_feedback(tmp_path):
    evidence = tmp_path / "contact_onsets.csv"
    write_onset_csv(evidence)
    report = analyze_feedback_onsets(evidence, 8.1, 0.0101)
    assert report["passed"]
    assert report["aggregate"]["evaluable_contact_activations"] == 3
    assert report["aggregate"]["max_rise_time_ms"] == pytest.approx(8.0)
    assert report["aggregate"]["max_torque_step_nm"] == pytest.approx(0.01)
    output = tmp_path / "onset.json"
    assert onset_main(
        [
            "--csv", str(evidence),
            "--max-rise-time-ms", "8.1",
            "--max-torque-step-nm", "0.0101",
            "--output", str(output),
        ]
    ) == 0
    assert json.loads(output.read_text())["passed"]
    step_rejected = analyze_feedback_onsets(evidence, 8.1, 0.009)
    assert (
        "CONTACT at row 3: torque step exceeded the limit"
        in step_rejected["failures"]
    )

    with evidence.open() as stream:
        rows = list(csv.DictReader(stream))
    rows[0]["tau_fb_j1"] = "0.001"
    with evidence.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    rejected = analyze_feedback_onsets(evidence, 8.1, 0.0101)
    assert not rejected["passed"]
    assert (
        "feedback torque is nonzero while canonical feedback is blocked"
        in rejected["failures"]
    )


def test_authorization_recomputes_bound_off_and_40_percent_csv(tmp_path):
    model = write_model(tmp_path / "model")
    limits = {**SAFETY_LIMITS, "max_contact_force_n": 5.0}
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
