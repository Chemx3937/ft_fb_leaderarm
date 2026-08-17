import hashlib
import json

from ft_fb_leaderarm.il_episode_verification import ARRAYS, analyze_il_episode


def _write_array(episode, relative, shape):
    target = episode / relative
    target.mkdir(parents=True)
    (target / ".zarray").write_text(json.dumps({"shape": list(shape)}))
    (target / "0").write_bytes(b"chunk")


def _episode(tmp_path):
    episode = tmp_path / "episode_000"
    episode.mkdir()
    model = tmp_path / "model.ts"
    model.write_bytes(b"approved model")
    model_hash = hashlib.sha256(model.read_bytes()).hexdigest()
    (episode / "meta.json").write_text(json.dumps({
        "model_sha256": model_hash,
        "feedback_gain_scale_contract": 0.40,
        "writer_error": None,
        "interruption_reason": None,
    }))
    for relative, tail in ARRAYS.values():
        _write_array(episode, relative, (100,) + tail)
    return episode, model


def test_il_episode_requires_complete_ft_feedback_contract(tmp_path):
    episode, model = _episode(tmp_path)
    report = analyze_il_episode(episode, model, 0.40)
    assert report["passed"]
    assert report["arrays"]["free_space_prediction"]["shape"] == [100, 6]

    prediction = episode / ARRAYS["free_space_prediction"][0] / ".zarray"
    prediction.unlink()
    (episode / "meta.json").write_text("{}")
    rejected = analyze_il_episode(episode, model, 0.40)
    assert not rejected["passed"]
    assert any("free_space_prediction" in value for value in rejected["failures"])
    assert "episode model_sha256 does not match the selected model" in rejected["failures"]
    assert "episode feedback stage is missing or mismatched" in rejected["failures"]
