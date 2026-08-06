import numpy as np
import pytest

from ft_fb_leaderarm.feedback_authorization import (
    scaled_leader_overrides,
    stage_for_scale,
)


def test_only_40_and_100_percent_stages_are_allowed():
    assert stage_for_scale(0.40) == 1
    assert stage_for_scale(1.00) == 2
    with pytest.raises(RuntimeError, match="0.40 or 1.00"):
        stage_for_scale(0.20)


def test_feedback_off_keeps_observer_but_zeros_gain(tmp_path):
    config = tmp_path / "leader.yaml"
    config.write_text(
        "leader_teleop_node:\n"
        "  ros__parameters:\n"
        "    right_jt_wrench_fb_gain: [1, 1, 1, 1, 1, 1]\n"
        "    jt_wrench_fb_clip: [1, 1, 1, 1, 1, 1]\n"
        "    tau_fb_slew_rate_Nm_s: [1, 1, 1, 1, 1, 1]\n"
    )
    assert scaled_leader_overrides(config, "right", 0.0) == {
        "right_jt_wrench_fb_gain": [0.0] * 6
    }
    enabled = scaled_leader_overrides(config, "right", 0.40)
    assert np.allclose(
        enabled["right_jt_wrench_fb_gain"],
        [0.0048, 0.0048, 0.026, 0.0048, 0.026, 0.0048],
    )
    assert enabled["tau_fb_motion_gate_enable"] is True
