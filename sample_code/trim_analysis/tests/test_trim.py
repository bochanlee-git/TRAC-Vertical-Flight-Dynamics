import numpy as np

from trim_example import (
    FlightCondition,
    PublicTrimModel,
    run_speed_sweep,
)


def test_hover_converges():
    model = PublicTrimModel()
    result = model.solve(
        FlightCondition(
            speed_kt=0.0
        )
    )
    assert result.converged
    assert 0.04 < result.lambda0 < 0.08


def test_60kt_converges():
    model = PublicTrimModel()
    result = model.solve(
        FlightCondition(
            speed_kt=60.0
        )
    )
    assert result.converged
    assert result.max_scaled_residual < 1.0e-7


def test_full_speed_sweep_converges():
    model = PublicTrimModel()
    results = run_speed_sweep(
        model,
        np.arange(
            0.0,
            161.0,
            20.0,
        ),
    )
    assert all(
        result.converged
        for _, result in results
    )


def test_mean_inflow_decreases_with_speed():
    model = PublicTrimModel()
    results = run_speed_sweep(
        model,
        [0.0, 40.0, 80.0, 120.0, 160.0],
    )
    lambdas = [
        result.lambda0
        for _, result in results
    ]
    assert all(
        a > b
        for a, b in zip(
            lambdas,
            lambdas[1:],
        )
    )


def test_high_speed_power_recovers():
    model = PublicTrimModel()
    results = run_speed_sweep(
        model,
        [60.0, 80.0, 120.0, 160.0],
    )
    powers = {
        condition.speed_kt:
            result.total_power_kw
        for condition, result in results
    }
    assert powers[160.0] > powers[80.0]
    assert powers[160.0] > powers[120.0]
