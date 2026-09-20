"""
Descriptor-form linearization example for the TRAC public repository.

This file demonstrates the linearization procedure documented in
Lee (2021), Section 2.5:

    f(y, y_dot, u) = 0

    E = df/dy_dot
    F = df/dy
    G = df/du

    A = -E^{-1} F
    B = -E^{-1} G

The numerical model below is an intentionally small, illustrative
longitudinal rotorcraft surrogate. It is NOT the original TRAC
UH-60 dynamic model and its eigenvalues must not be interpreted as
UH-60 flight-dynamic modes.

The purpose is to expose a reusable Jacobian-extraction workflow that
can later be connected to a higher-fidelity nonlinear model.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse

import numpy as np


STATE_NAMES = (
    "u",
    "w",
    "q",
    "theta",
    "lambda",
)

CONTROL_NAMES = (
    "collective",
    "longitudinal_cyclic",
)


@dataclass(frozen=True)
class LinearModel:
    """Container for the descriptor and state-space matrices."""

    E: np.ndarray
    F: np.ndarray
    G: np.ndarray
    A: np.ndarray
    B: np.ndarray
    eigenvalues: np.ndarray


class IllustrativeLongitudinalModel:
    """
    Small nonlinear implicit model used only to demonstrate linearization.

    States
    ------
    u       : forward-speed perturbation [normalized]
    w       : vertical-speed perturbation [normalized]
    q       : pitch-rate perturbation [rad/s]
    theta   : pitch-angle perturbation [rad]
    lambda  : mean-inflow perturbation [normalized]

    Controls
    --------
    collective            : normalized collective perturbation
    longitudinal_cyclic   : normalized longitudinal-cyclic perturbation

    Notes
    -----
    Coefficients in this illustrative system are not identified from the
    UH-60 flight-test data and are not original TRAC coefficients.
    """

    def residual(
        self,
        y: np.ndarray,
        y_dot: np.ndarray,
        control: np.ndarray,
    ) -> np.ndarray:
        y = np.asarray(y, dtype=float)
        y_dot = np.asarray(y_dot, dtype=float)
        control = np.asarray(control, dtype=float)

        u, w, q, theta, inflow = y
        u_dot, w_dot, q_dot, theta_dot, inflow_dot = y_dot
        collective, cyclic = control

        # Explicit nonlinear right-hand side.
        # Mild nonlinear terms are included so the finite-difference
        # linearization is nontrivial.
        rhs_u = (
            -0.08 * u
            + 0.10 * w
            - 0.65 * theta
            - 0.18 * inflow
            + 0.30 * collective
            + 0.015 * u * w
        )

        rhs_w = (
            -0.04 * u
            - 0.32 * w
            + 0.55 * q
            - 0.60 * inflow
            + 0.85 * collective
            - 0.010 * u**2
        )

        rhs_q = (
            0.015 * u
            - 0.06 * w
            - 0.55 * q
            - 0.80 * theta
            + 0.95 * cyclic
            + 0.020 * u * theta
        )

        rhs_theta = q

        rhs_inflow = (
            -1.80 * inflow
            + 0.65 * collective
            - 0.15 * u
            - 0.10 * w
            + 0.025 * collective * u
        )

        # Descriptor form. Non-unity coefficients are deliberate so that
        # E is not simply the identity matrix.
        return np.array(
            [
                1.00 * u_dot - rhs_u,
                1.15 * w_dot - rhs_w,
                0.80 * q_dot - rhs_q,
                1.00 * theta_dot - rhs_theta,
                0.45 * inflow_dot - rhs_inflow,
            ],
            dtype=float,
        )


def finite_difference_jacobian(
    function,
    x0: np.ndarray,
    relative_step: float = 1.0e-6,
) -> np.ndarray:
    """
    Central finite-difference Jacobian.

    The step for each variable is:

        h_i = relative_step * max(1, |x_i|)
    """

    x0 = np.asarray(
        x0,
        dtype=float,
    )

    f0 = np.asarray(
        function(x0),
        dtype=float,
    )

    jacobian = np.zeros(
        (
            f0.size,
            x0.size,
        ),
        dtype=float,
    )

    for i in range(
        x0.size
    ):
        step = (
            relative_step
            * max(
                1.0,
                abs(x0[i]),
            )
        )

        xp = x0.copy()
        xm = x0.copy()

        xp[i] += step
        xm[i] -= step

        fp = np.asarray(
            function(xp),
            dtype=float,
        )

        fm = np.asarray(
            function(xm),
            dtype=float,
        )

        jacobian[:, i] = (
            fp - fm
        ) / (
            2.0 * step
        )

    return jacobian


def extract_linear_model(
    residual_function,
    y_trim: np.ndarray,
    y_dot_trim: np.ndarray,
    control_trim: np.ndarray,
    relative_step: float = 1.0e-6,
) -> LinearModel:
    """
    Extract E, F, G, A, and B from an implicit nonlinear system.

    This directly mirrors Lee (2021), Eqs. (2.82)-(2.84).
    """

    y_trim = np.asarray(
        y_trim,
        dtype=float,
    )

    y_dot_trim = np.asarray(
        y_dot_trim,
        dtype=float,
    )

    control_trim = np.asarray(
        control_trim,
        dtype=float,
    )

    E = finite_difference_jacobian(
        lambda value: residual_function(
            y_trim,
            value,
            control_trim,
        ),
        y_dot_trim,
        relative_step,
    )

    F = finite_difference_jacobian(
        lambda value: residual_function(
            value,
            y_dot_trim,
            control_trim,
        ),
        y_trim,
        relative_step,
    )

    G = finite_difference_jacobian(
        lambda value: residual_function(
            y_trim,
            y_dot_trim,
            value,
        ),
        control_trim,
        relative_step,
    )

    # Avoid explicitly forming E^{-1}.
    A = -np.linalg.solve(
        E,
        F,
    )

    B = -np.linalg.solve(
        E,
        G,
    )

    eigenvalues = np.linalg.eigvals(
        A
    )

    return LinearModel(
        E=E,
        F=F,
        G=G,
        A=A,
        B=B,
        eigenvalues=eigenvalues,
    )


def linear_prediction_error(
    model: IllustrativeLongitudinalModel,
    linear_model: LinearModel,
    delta_y: np.ndarray,
    delta_u: np.ndarray,
) -> float:
    """
    Compare the nonlinear residual-derived state derivative with A dx+B du.

    At the perturbation equilibrium used here, the descriptor equations are
    solved for y_dot numerically and compared with the linear prediction.
    """

    delta_y = np.asarray(
        delta_y,
        dtype=float,
    )

    delta_u = np.asarray(
        delta_u,
        dtype=float,
    )

    # Since the example residual is affine in y_dot, solve:
    # E * y_dot + r(y, 0, u) = 0
    residual_at_zero_derivative = model.residual(
        delta_y,
        np.zeros_like(
            delta_y
        ),
        delta_u,
    )

    nonlinear_y_dot = -np.linalg.solve(
        linear_model.E,
        residual_at_zero_derivative,
    )

    linear_y_dot = (
        linear_model.A @ delta_y
        + linear_model.B @ delta_u
    )

    return float(
        np.linalg.norm(
            nonlinear_y_dot
            - linear_y_dot,
        )
    )


def save_outputs(
    linear_model: LinearModel,
    output_dir: Path,
) -> None:
    """Save matrices and eigenvalues as CSV files."""

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    np.savetxt(
        output_dir / "E.csv",
        linear_model.E,
        delimiter=",",
    )

    np.savetxt(
        output_dir / "F.csv",
        linear_model.F,
        delimiter=",",
    )

    np.savetxt(
        output_dir / "G.csv",
        linear_model.G,
        delimiter=",",
    )

    np.savetxt(
        output_dir / "A.csv",
        linear_model.A,
        delimiter=",",
    )

    np.savetxt(
        output_dir / "B.csv",
        linear_model.B,
        delimiter=",",
    )

    eigen_table = np.column_stack(
        (
            linear_model.eigenvalues.real,
            linear_model.eigenvalues.imag,
        )
    )

    np.savetxt(
        output_dir / "eigenvalues.csv",
        eigen_table,
        delimiter=",",
        header="real,imag",
        comments="",
    )


def print_matrix(
    name: str,
    matrix: np.ndarray,
) -> None:
    print()
    print(name)
    print(
        np.array2string(
            matrix,
            precision=5,
            suppress_small=True,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Descriptor-form finite-difference linearization example."
        )
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
        help="Save E/F/G/A/B and eigenvalues to linearization_results/.",
    )

    args = parser.parse_args()

    nonlinear_model = IllustrativeLongitudinalModel()

    # Perturbation equilibrium.
    y_trim = np.zeros(
        len(STATE_NAMES)
    )

    y_dot_trim = np.zeros_like(
        y_trim
    )

    control_trim = np.zeros(
        len(CONTROL_NAMES)
    )

    trim_residual = nonlinear_model.residual(
        y_trim,
        y_dot_trim,
        control_trim,
    )

    linear_model = extract_linear_model(
        residual_function=nonlinear_model.residual,
        y_trim=y_trim,
        y_dot_trim=y_dot_trim,
        control_trim=control_trim,
        relative_step=args.step,
    )

    print(
        "Equilibrium residual norm: "
        f"{np.linalg.norm(trim_residual):.3e}"
    )

    print_matrix(
        "E = df/dy_dot",
        linear_model.E,
    )

    print_matrix(
        "F = df/dy",
        linear_model.F,
    )

    print_matrix(
        "G = df/du",
        linear_model.G,
    )

    print_matrix(
        "A = -E^-1 F",
        linear_model.A,
    )

    print_matrix(
        "B = -E^-1 G",
        linear_model.B,
    )

    print()
    print("Eigenvalues of A")

    for value in sorted(
        linear_model.eigenvalues,
        key=lambda item: (
            item.real,
            item.imag,
        ),
    ):
        print(
            f"{value.real:+.6f} "
            f"{value.imag:+.6f}j"
        )

    # Demonstrate first-order local accuracy.
    direction_y = np.array(
        [
            0.5,
            -0.2,
            0.1,
            0.05,
            0.15,
        ]
    )

    direction_u = np.array(
        [
            0.2,
            -0.1,
        ]
    )

    print()
    print("Local linear-prediction error")

    for scale in (
        1.0e-1,
        5.0e-2,
        1.0e-2,
        5.0e-3,
    ):
        error = linear_prediction_error(
            nonlinear_model,
            linear_model,
            scale * direction_y,
            scale * direction_u,
        )

        print(
            f"scale={scale:.1e} "
            f"error={error:.3e}"
        )

    if args.save:
        output_dir = (
            Path(__file__).resolve().parent
            / "linearization_results"
        )

        save_outputs(
            linear_model,
            output_dir,
        )

        print()
        print(
            f"Saved results to: {output_dir}"
        )


if __name__ == "__main__":
    main()
