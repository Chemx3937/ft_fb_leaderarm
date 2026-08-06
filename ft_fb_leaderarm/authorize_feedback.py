"""Create one human-attested, artifact-bound FT feedback stage authorization."""

import argparse

from .feedback_authorization import create_feedback_authorization


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-path", required=True)
    parser.add_argument("--gain-scale", required=True, type=float)
    parser.add_argument("--evidence", required=True, nargs="+")
    parser.add_argument("--previous-authorization", default="")
    parser.add_argument("--operator-attestation", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    output = create_feedback_authorization(
        args.output,
        args.model_path,
        args.gain_scale,
        args.evidence,
        args.operator_attestation,
        args.previous_authorization,
    )
    print(output)
