import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
SIMULATION_DIR = HERE.parent

if str(SIMULATION_DIR) not in sys.path:
    sys.path.insert(
        0,
        str(SIMULATION_DIR),
    )


from simulation_example import (  # noqa: E402
    build_state_space_model,
    controllability_matrix,
    run_lqr_tracking,
    run_open_loop,
    steady_state_for_output,
)


def test_model_is_fully_controllable():
    model = build_state_space_model()

    controllability = controllability_matrix(
        model.A,
        model.B,
    )

    assert np.linalg.matrix_rank(
        controllability
    ) == model.A.shape[0]


def test_lqr_closed_loop_is_stable():
    model = build_state_space_model()

    (
        _time,
        _states,
        _controls,
        lqr,
        _C,
        _x_ss,
        _u_ss,
    ) = run_lqr_tracking(
        model
    )

    assert np.all(
        lqr.closed_loop_eigenvalues.real
        < 0.0
    )


def test_steady_state_equations_are_satisfied():
    model = build_state_space_model()

    C = np.array(
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
        ]
    )

    reference = np.array(
        [
            0.15,
            0.05,
        ]
    )

    x_ss, u_ss = steady_state_for_output(
        model.A,
        model.B,
        C,
        reference,
    )

    assert np.linalg.norm(
        model.A @ x_ss
        + model.B @ u_ss
    ) < 1.0e-10

    assert np.allclose(
        C @ x_ss,
        reference,
        atol=1.0e-10,
    )


def test_tracking_error_reduces():
    model = build_state_space_model()

    (
        _time,
        states,
        _controls,
        _lqr,
        C,
        x_ss,
        _u_ss,
    ) = run_lqr_tracking(
        model
    )

    target = C @ x_ss

    initial_error = np.linalg.norm(
        C @ states[:, 0]
        - target
    )

    final_error = np.linalg.norm(
        C @ states[:, -1]
        - target
    )

    assert final_error < initial_error
    assert final_error < 1.0e-3


def test_open_loop_output_is_finite():
    model = build_state_space_model()

    time, states, controls = run_open_loop(
        model
    )

    assert np.all(
        np.isfinite(
            time
        )
    )

    assert np.all(
        np.isfinite(
            states
        )
    )

    assert np.all(
        np.isfinite(
            controls
        )
    )
