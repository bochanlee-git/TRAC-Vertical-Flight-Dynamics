import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
LINEARIZATION_DIR = HERE.parent
TRIM_DIR = LINEARIZATION_DIR.parent / "trim_analysis"

for path in (
    LINEARIZATION_DIR,
    TRIM_DIR,
):
    if str(path) not in sys.path:
        sys.path.insert(
            0,
            str(path),
        )


from linearize_example import (  # noqa: E402
    IllustrativeLongitudinalModel,
    extract_linear_model,
    finite_difference_jacobian,
    linear_prediction_error,
)

from trim_example import (  # noqa: E402
    FlightCondition,
    PublicTrimModel,
)


def make_linear_model():
    nonlinear_model = IllustrativeLongitudinalModel()

    y = np.zeros(5)
    y_dot = np.zeros(5)
    control = np.zeros(2)

    linear_model = extract_linear_model(
        nonlinear_model.residual,
        y,
        y_dot,
        control,
    )

    return (
        nonlinear_model,
        linear_model,
    )


def test_equilibrium_residual_is_zero():
    model = IllustrativeLongitudinalModel()

    residual = model.residual(
        np.zeros(5),
        np.zeros(5),
        np.zeros(2),
    )

    assert np.linalg.norm(residual) < 1.0e-14


def test_descriptor_matrix_recovered():
    _, linear_model = make_linear_model()

    expected = np.diag(
        [
            1.00,
            1.15,
            0.80,
            1.00,
            0.45,
        ]
    )

    assert np.allclose(
        linear_model.E,
        expected,
        atol=1.0e-9,
        rtol=1.0e-9,
    )


def test_state_space_identity():
    _, linear_model = make_linear_model()

    assert np.allclose(
        linear_model.E @ linear_model.A
        + linear_model.F,
        0.0,
        atol=1.0e-10,
    )

    assert np.allclose(
        linear_model.E @ linear_model.B
        + linear_model.G,
        0.0,
        atol=1.0e-10,
    )


def test_linear_prediction_improves_with_smaller_perturbation():
    nonlinear_model, linear_model = make_linear_model()

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

    large_error = linear_prediction_error(
        nonlinear_model,
        linear_model,
        0.1 * direction_y,
        0.1 * direction_u,
    )

    small_error = linear_prediction_error(
        nonlinear_model,
        linear_model,
        0.01 * direction_y,
        0.01 * direction_u,
    )

    assert small_error < large_error


def test_public_trim_jacobian_is_full_rank():
    trim_model = PublicTrimModel()

    condition = FlightCondition(
        speed_kt=60.0
    )

    result = trim_model.solve(
        condition
    )

    assert result.converged

    x_trim = np.array(
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

    jacobian = finite_difference_jacobian(
        lambda value: trim_model.residuals(
            value,
            condition,
        ),
        x_trim,
    )

    assert jacobian.shape == (8, 8)
    assert np.linalg.matrix_rank(jacobian) == 8
