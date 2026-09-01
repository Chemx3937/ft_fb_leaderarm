"""Right-hand measurement contract used by the IL data recorder."""

import numpy as np


RIGHT_HAND_JOINT_NAMES = (
    "right_thumb_joint1",
    "right_thumb_joint2",
    "right_thumb_joint3",
    "right_index_joint1",
    "right_index_joint2",
    "right_index_joint3",
    "right_middle_joint1",
    "right_middle_joint2",
    "right_middle_joint3",
    "right_ring_joint1",
    "right_ring_joint2",
    "right_ring_joint3",
    "right_baby_joint1",
    "right_baby_joint2",
    "right_baby_joint3",
)


def validate_right_hand_joint_measurements(
    values,
    *,
    label="measured right hand joint values",
):
    """Return finite ``(..., 15)`` measured joint radians."""
    array = np.asarray(values)
    expected = len(RIGHT_HAND_JOINT_NAMES)
    if array.ndim < 1 or array.shape[-1] != expected:
        raise ValueError(
            f"{label} must have trailing shape ({expected},), got {array.shape}"
        )
    if not np.issubdtype(array.dtype, np.number):
        raise ValueError(f"{label} must use a numeric dtype, got {array.dtype}")
    numeric = np.asarray(array, dtype=np.float64)
    if not np.isfinite(numeric).all():
        raise ValueError(f"{label} contains non-finite values")
    return numeric
