from pathlib import Path
import sys


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import compare_free_space_wrench_runtime_residuals as comparison


def test_runtime_residual_comparison_self_check():
    comparison.self_check()
