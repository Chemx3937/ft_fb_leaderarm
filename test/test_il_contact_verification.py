from ft_fb_leaderarm.il_contact_verification import (
    FRAME,
    MESSAGE_TYPE,
    TOPIC,
    analyze_il_contact_contract,
)


def _configs():
    recorder = {
        "record_contact_observation": True,
        "contact_observation_topic": TOPIC,
        "observer_input_frame_id": FRAME,
        "contact_observation_hz": 262.5,
    }
    policy = {
        "topics": {
            "contact_observation": TOPIC,
            "contact_observation_frame_id": FRAME,
        },
        "preprocess": {"contact_state_hz": 262.5},
    }
    return recorder, policy


def test_collection_and_inference_share_one_canonical_publisher():
    recorder, policy = _configs()
    publisher = [{"node": "/ft_contact_observer", "topic_type": MESSAGE_TYPE}]
    sample = [{
        "frame_id": FRAME,
        "contact_state": 0,
        "contact_wrench": [0.0] * 6,
        "valid": True,
        "model_ready": True,
    }]
    collection = analyze_il_contact_contract(
        recorder,
        policy,
        "collection",
        publisher,
        [{"node": "/chem_acp_raw_data_collection", "topic_type": MESSAGE_TYPE}],
        sample,
    )
    inference = analyze_il_contact_contract(
        recorder,
        policy,
        "inference",
        publisher,
        [{"node": "/chem_acp_env_runner", "topic_type": MESSAGE_TYPE}],
        sample,
    )
    assert collection["passed"]
    assert inference["passed"]

    duplicated = analyze_il_contact_contract(
        recorder,
        policy,
        "collection",
        publisher * 2,
        [{"node": "/chem_acp_raw_data_collection", "topic_type": MESSAGE_TYPE}],
        sample,
    )
    assert not duplicated["passed"]
    assert "canonical contact topic must have exactly one publisher" in duplicated[
        "failures"
    ]
