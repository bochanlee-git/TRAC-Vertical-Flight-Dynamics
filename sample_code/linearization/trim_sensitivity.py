"""
Local sensitivity analysis of the public trim residual system.

This script connects the linearization folder to the public
sample_code/trim_analysis model.

It computes:

    J_trim = dR_trim / dx_trim

at a converged trim point using central finite differences.

This is NOT the aircraft flight-dynamics A matrix.

It is the local Jacobian of the nonlinear trim equations and is useful for:
- checking local conditioning,
- identifying sensitive directions,
- diagnosing solver robustness,
- and verifying the finite-difference infrastructure before applying it
  to a full dynamic residual f(y, y_dot, u).
"""

from __future__ import annotations

from pathlib import Path
import argparse
import sys

import numpy as np


HERE = Path(__file__).resolve().parent
TRIM_DIR = HERE.parent / "trim_analysis"

if str(TRIM_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(TRIM_DIR),
    )

from trim_example import (  # noqa: E402
    FlightCondition,
    PublicTrimModel,
)

from linearize_example import (  # noqa: E402
    finite_difference_jacobian,
)


TRIM_VARIABLE_NAMES = (
    "theta0_deg",
    "theta1s_deg",
    "beta0_deg",
    "beta1c_deg",
    "beta1s_deg",
    "lambda0",
    "lambda1c",
    "lambda1s",
)


def trim_vector_from_result(
    result,
) -> np.ndarray:
    return np.array(
        [
            result.collective_deg,
            result.longitudinal_cyclic_deg,
            result.beta0_deg,
            result.beta1c_deg,
            result.beta1s_deg,
            result.lambda0,
            result.lambda1c,
            result.lambda1s,
        ],
        dtype=float,
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Finite-difference Jacobian of the public trim residual system."
        )
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=60.0,
        help="Forward speed in knots.",
    )

    parser.add_argument(
        "--step",
        type=float,
        default=1.0e-6,
        help="Relative finite-difference step.",
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save the Jacobian and singular values as CSV files.",
    )

    args = parser.parse_args()

    model = PublicTrimModel()

    condition = FlightCondition(
        speed_kt=args.speed
    )

    trim_result = model.solve(
        condition
    )

    if not trim_result.converged:
        raise RuntimeError(
            "Trim solution did not converge."
        )

    x_trim = trim_vector_from_result(
        trim_result
    )

    jacobian = finite_difference_jacobian(
        lambda value: model.residuals(
            value,
            condition,
        ),
        x_trim,
        relative_step=args.step,
    )

    singular_values = np.linalg.svd(
        jacobian,
        compute_uv=False,
    )

    condition_number = np.linalg.cond(
        jacobian
    )

    rank = np.linalg.matrix_rank(
        jacobian
    )

    residual_norm = np.linalg.norm(
        model.residuals(
            x_trim,
            condition,
        )
    )

    print(
        f"Trim speed: {args.speed:.1f} kt"
    )

    print(
        f"Trim residual norm: {residual_norm:.3e}"
    )

    print(
        f"Jacobian shape: {jacobian.shape}"
    )

    print(
        f"Jacobian rank: {rank}"
    )

    print(
        f"2-norm condition number: {condition_number:.3e}"
    )

    print()
    print("Singular values")

    for value in singular_values:
        print(
            f"{value:.6e}"
        )

    print()
    print("Trim Jacobian")

    print(
        np.array2string(
            jacobian,
            precision=5,
            suppress_small=True,
        )
    )

    if args.save:
        output_dir = (
            HERE
            / "trim_sensitivity_results"
        )

        output_dir.mkdir(
            exist_ok=True
        )

        np.savetxt(
            output_dir / "trim_jacobian.csv",
            jacobian,
            delimiter=",",
        )

        np.savetxt(
            output_dir / "singular_values.csv",
            singular_values,
            delimiter=",",
        )

        with (
            output_dir / "variable_names.txt"
        ).open(
            "w",
            encoding="utf-8",
        ) as stream:
            stream.write(
                "\n".join(
                    TRIM_VARIABLE_NAMES
                )
            )

        print()
        print(
            f"Saved results to: {output_dir}"
        )


if __name__ == "__main__":
    main()
