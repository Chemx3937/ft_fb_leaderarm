import csv

import pytest

from ft_fb_leaderarm.contact_evaluation import analyze_contact_evidence


def _write_observations(path, predicted):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "t_s", "observer_contact_state", "observer_valid",
                "observer_model_ready",
            ),
        )
        writer.writeheader()
        for index, contact in enumerate(predicted):
            writer.writerow(
                {
                    "t_s": index * 0.01,
                    "observer_contact_state": int(contact),
                    "observer_valid": 1,
                    "observer_model_ready": 1,
                }
            )


def _write_truth(path):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("start_s", "end_s"))
        writer.writeheader()
        writer.writerow({"start_s": 0.025, "end_s": 0.075})


def test_contact_metrics_and_false_contact_gate(tmp_path):
    observations = tmp_path / "observations.csv"
    truth = tmp_path / "ground_truth.csv"
    _write_truth(truth)
    predicted = [False] * 10
    predicted[4:8] = [True] * 4
    _write_observations(observations, predicted)

    report = analyze_contact_evidence(
        observations, truth, 1.0, 0.8, 15.1, 5.1
    )
    assert report["passed"]
    assert report["metrics"]["precision"] == 1.0
    assert report["metrics"]["recall"] == 0.8
    assert report["metrics"]["onset_latency_max_ms"] == pytest.approx(15.0)
    assert report["metrics"]["release_latency_max_ms"] == pytest.approx(5.0)

    predicted[1] = True
    _write_observations(observations, predicted)
    rejected = analyze_contact_evidence(
        observations, truth, 0.8, 0.8, 15.1, 5.1
    )
    assert not rejected["passed"]
    assert rejected["metrics"]["false_contact_activations"] == 1
    assert "FREE samples contain false CONTACT predictions" in rejected["failures"]


def test_ground_truth_events_require_a_free_gap(tmp_path):
    observations = tmp_path / "observations.csv"
    truth = tmp_path / "ground_truth.csv"
    _write_observations(observations, [False] * 10)
    with truth.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("start_s", "end_s"))
        writer.writeheader()
        writer.writerow({"start_s": 0.02, "end_s": 0.04})
        writer.writerow({"start_s": 0.04, "end_s": 0.06})

    with pytest.raises(RuntimeError, match="overlap, touch, or are unordered"):
        analyze_contact_evidence(observations, truth, 0.9, 0.9, 20.0, 30.0)
