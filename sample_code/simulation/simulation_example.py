"""
Open-loop, forced-response, and LQR simulation examples.

This file uses the illustrative A/B matrices generated from
sample_code/linearization/linearize_example.py.

It is NOT a UH-60 trajectory simulation.

The purpose is to demonstrate the public repository workflow:

    nonlinear model
        -> linearization
        -> A / B matrices
        -> open-loop response
        -> forced response
        -> LQR feedback simulation
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
import argparse

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are


HERE = Path(__file__).resolve().parent
LINEARIZATION_DIR = HERE.parent / "linearization"

if str(LINEARIZATION_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(LINEARIZATION_DIR),
    )

from linearize_example import (  # noqa: E402
    CONTROL_NAMES,
    STATE_NAMES,
    IllustrativeLongitudinalModel,
    extract_linear_model,
)


@dataclass(frozen=True)
class StateSpaceModel:
    A: np.ndarray
    B: np.ndarray
    state_names: tuple[str, ...]
    control_names: tuple[str, ...]


@dataclass(frozen=True)
class LQRDesign:
    K: np.ndarray
    P: np.ndarray
    closed_loop_eigenvalues: np.ndarray


def build_state_space_model() -> StateSpaceModel:
    """
    Recreate the illustrative linearized model used in the linearization sample.
    """

    nonlinear_model = IllustrativeLongitudinalModel()

    y_trim = np.zeros(
        len(STATE_NAMES)
    )

    y_dot_trim = np.zeros_like(
        y_trim
    )

    control_trim = np.zeros(
        len(CONTROL_NAMES)
    )

    linear_model = extract_linear_model(
        residual_function=nonlinear_model.residual,
        y_trim=y_trim,
        y_dot_trim=y_dot_trim,
        control_trim=control_trim,
    )

    return StateSpaceModel(
        A=linear_model.A,
        B=linear_model.B,
        state_names=STATE_NAMES,
        control_names=CONTROL_NAMES,
    )


def controllability_matrix(
    A: np.ndarray,
    B: np.ndarray,
) -> np.ndarray:
    """
    Construct [B, AB, A^2B, ...].
    """

    n = A.shape[0]

    blocks = [
        B
    ]

    current = B.copy()

    for _ in range(
        1,
        n,
    ):
        current = A @ current
        blocks.append(
            current
        )

    return np.hstack(
        blocks
    )


def design_lqr(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
) -> LQRDesign:
    """
    Continuous-time infinite-horizon LQR.

    Solves the continuous algebraic Riccati equation and returns:

        u = -K x
    """

    P = solve_continuous_are(
        A,
        B,
        Q,
        R,
    )

    K = np.linalg.solve(
        R,
        B.T @ P,
    )

    closed_loop_eigenvalues = np.linalg.eigvals(
        A - B @ K
    )

    return LQRDesign(
        K=K,
        P=P,
        closed_loop_eigenvalues=closed_loop_eigenvalues,
    )


def steady_state_for_output(
    A: np.ndarray,
    B: np.ndarray,
    C: np.ndarray,
    y_reference: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Solve the set-point equilibrium described by the block system:

        A x_ss + B u_ss = 0
        C x_ss           = y_reference

    This follows the structure used for set-point tracking in Lee (2021).

    The number of selected outputs must equal the number of controls so the
    block matrix is square.
    """

    A = np.asarray(
        A,
        dtype=float,
    )

    B = np.asarray(
        B,
        dtype=float,
    )

    C = np.asarray(
        C,
        dtype=float,
    )

    y_reference = np.asarray(
        y_reference,
        dtype=float,
    )

    n = A.shape[0]
    m = B.shape[1]

    if C.shape != (
        m,
        n,
    ):
        raise ValueError(
            "C must contain one selected output for each control input."
        )

    block = np.block(
        [
            [
                A,
                B,
            ],
            [
                C,
                np.zeros(
                    (
                        m,
                        m,
                    )
                ),
            ],
        ]
    )

    rhs = np.concatenate(
        (
            np.zeros(
                n
            ),
            y_reference,
        )
    )

    solution = np.linalg.solve(
        block,
        rhs,
    )

    x_ss = solution[:n]
    u_ss = solution[n:]

    return (
        x_ss,
        u_ss,
    )


def simulate_linear_system(
    A: np.ndarray,
    B: np.ndarray,
    x0: np.ndarray,
    control_law,
    t_final: float = 25.0,
    samples: int = 1001,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Integrate:

        x_dot = A x + B u(t, x)
    """

    x0 = np.asarray(
        x0,
        dtype=float,
    )

    time = np.linspace(
        0.0,
        t_final,
        samples,
    )

    def dynamics(
        t,
        x,
    ):
        u = np.asarray(
            control_law(
                t,
                x,
            ),
            dtype=float,
        )

        return (
            A @ x
            + B @ u
        )

    solution = solve_ivp(
        dynamics,
        (
            0.0,
            t_final,
        ),
        x0,
        t_eval=time,
        rtol=1.0e-9,
        atol=1.0e-11,
    )

    if not solution.success:
        raise RuntimeError(
            solution.message
        )

    controls = np.column_stack(
        [
            np.asarray(
                control_law(
                    t,
                    x,
                ),
                dtype=float,
            )
            for t, x in zip(
                solution.t,
                solution.y.T,
            )
        ]
    )

    return (
        solution.t,
        solution.y,
        controls,
    )


def run_open_loop(
    model: StateSpaceModel,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Zero-input response from a non-zero initial perturbation.
    """

    x0 = np.array(
        [
            0.20,
            -0.10,
            0.08,
            0.05,
            0.10,
        ],
        dtype=float,
    )

    return simulate_linear_system(
        A=model.A,
        B=model.B,
        x0=x0,
        control_law=lambda _t, _x: np.zeros(
            model.B.shape[1]
        ),
    )


def run_forced_response(
    model: StateSpaceModel,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Demonstrate the response to small step-like control perturbations.
    """

    x0 = np.zeros(
        model.A.shape[0]
    )

    def step_control(
        t,
        _x,
    ):
        if t < 1.0:
            return np.zeros(
                model.B.shape[1]
            )

        return np.array(
            [
                0.08,
                0.04,
            ],
            dtype=float,
        )

    return simulate_linear_system(
        A=model.A,
        B=model.B,
        x0=x0,
        control_law=step_control,
    )


def run_lqr_tracking(
    model: StateSpaceModel,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    LQRDesign,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    LQR set-point tracking example.

    The illustrative model has two controls, so two outputs are selected:
        w
        theta

    The dissertation UH-60 model used four controls and therefore could
    select four non-zero reference states in the corresponding block solve.
    """

    # Illustrative weighting matrices.
    Q = np.diag(
        [
            2.0,
            8.0,
            4.0,
            10.0,
            2.0,
        ]
    )

    R = np.diag(
        [
            1.0,
            1.0,
        ]
    )

    lqr = design_lqr(
        model.A,
        model.B,
        Q,
        R,
    )

    # Track w and theta.
    C_track = np.array(
        [
            [
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ],
        ],
        dtype=float,
    )

    y_reference = np.array(
        [
            0.15,
            0.05,
        ],
        dtype=float,
    )

    x_ss, u_ss = steady_state_for_output(
        model.A,
        model.B,
        C_track,
        y_reference,
    )

    x0 = np.zeros(
        model.A.shape[0]
    )

    def control_law(
        _t,
        x,
    ):
        return (
            u_ss
            - lqr.K
            @ (
                x - x_ss
            )
        )

    time, states, controls = simulate_linear_system(
        A=model.A,
        B=model.B,
        x0=x0,
        control_law=control_law,
        t_final=25.0,
    )

    return (
        time,
        states,
        controls,
        lqr,
        C_track,
        x_ss,
        u_ss,
    )


def settling_time(
    time: np.ndarray,
    signal: np.ndarray,
    target: float,
    tolerance_fraction: float = 0.02,
    absolute_floor: float = 1.0e-4,
) -> float:
    """
    Return the first time after which the signal remains inside the band.
    """

    tolerance = max(
        abs(target)
        * tolerance_fraction,
        absolute_floor,
    )

    inside = np.abs(
        signal - target
    ) <= tolerance

    for i in range(
        len(time)
    ):
        if np.all(
            inside[i:]
        ):
            return float(
                time[i]
            )

    return float(
        "nan"
    )


def save_csv(
    path: Path,
    time: np.ndarray,
    states: np.ndarray,
    controls: np.ndarray,
    state_names: tuple[str, ...],
    control_names: tuple[str, ...],
) -> None:
    """
    Save one simulation history without requiring pandas.
    """

    path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    data = np.column_stack(
        (
            time,
            states.T,
            controls.T,
        )
    )

    header = ",".join(
        (
            "time_s",
            *state_names,
            *control_names,
        )
    )

    np.savetxt(
        path,
        data,
        delimiter=",",
        header=header,
        comments="",
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Open-loop, forced-response, and LQR simulation examples."
        )
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save simulation histories as CSV files.",
    )

    args = parser.parse_args()

    model = build_state_space_model()

    controllability = controllability_matrix(
        model.A,
        model.B,
    )

    print(
        "Controllability rank: "
        f"{np.linalg.matrix_rank(controllability)} / {model.A.shape[0]}"
    )

    print()
    print("Open-loop eigenvalues")

    for value in np.linalg.eigvals(
        model.A
    ):
        print(
            f"{value.real:+.6f} "
            f"{value.imag:+.6f}j"
        )

    open_time, open_states, open_controls = run_open_loop(
        model
    )

    forced_time, forced_states, forced_controls = run_forced_response(
        model
    )

    (
        lqr_time,
        lqr_states,
        lqr_controls,
        lqr,
        C_track,
        x_ss,
        u_ss,
    ) = run_lqr_tracking(
        model
    )

    print()
    print("Closed-loop eigenvalues")

    for value in lqr.closed_loop_eigenvalues:
        print(
            f"{value.real:+.6f} "
            f"{value.imag:+.6f}j"
        )

    print()
    print("Steady-state target")

    print(
        "x_ss = "
        + np.array2string(
            x_ss,
            precision=6,
            suppress_small=True,
        )
    )

    print(
        "u_ss = "
        + np.array2string(
            u_ss,
            precision=6,
            suppress_small=True,
        )
    )

    tracked_output = (
        C_track @ lqr_states
    )

    final_error = (
        tracked_output[:, -1]
        - C_track @ x_ss
    )

    print()
    print(
        "Final tracked-output error = "
        + np.array2string(
            final_error,
            precision=8,
            suppress_small=True,
        )
    )

    w_settle = settling_time(
        lqr_time,
        tracked_output[0],
        (C_track @ x_ss)[0],
    )

    theta_settle = settling_time(
        lqr_time,
        tracked_output[1],
        (C_track @ x_ss)[1],
    )

    print(
        f"w 2% settling time: {w_settle:.3f} s"
    )

    print(
        f"theta 2% settling time: {theta_settle:.3f} s"
    )

    print()
    print(
        "Peak absolute LQR control = "
        + np.array2string(
            np.max(
                np.abs(
                    lqr_controls
                ),
                axis=1,
            ),
            precision=6,
        )
    )

    if args.save:
        output_dir = (
            HERE
            / "simulation_results"
        )

        save_csv(
            output_dir / "open_loop.csv",
            open_time,
            open_states,
            open_controls,
            model.state_names,
            model.control_names,
        )

        save_csv(
            output_dir / "forced_response.csv",
            forced_time,
            forced_states,
            forced_controls,
            model.state_names,
            model.control_names,
        )

        save_csv(
            output_dir / "lqr_tracking.csv",
            lqr_time,
            lqr_states,
            lqr_controls,
            model.state_names,
            model.control_names,
        )

        np.savetxt(
            output_dir / "lqr_gain_K.csv",
            lqr.K,
            delimiter=",",
        )

        np.savetxt(
            output_dir / "x_ss.csv",
            x_ss,
            delimiter=",",
        )

        np.savetxt(
            output_dir / "u_ss.csv",
            u_ss,
            delimiter=",",
        )

        print()
        print(
            f"Saved results to: {output_dir}"
        )


if __name__ == "__main__":
    main()
