"""
Public UH-60 trim example for the TRAC portfolio repository.

This is NOT the original TRAC source code.

Design goal
-----------
Provide a compact, executable, source-informed trim example that exposes
the main numerical workflow without pretending to reproduce the complete
research implementation.

The model solves a longitudinal forward-flight problem with:
    - collective pitch theta_0
    - longitudinal cyclic theta_1s
    - first-harmonic flapping beta_0, beta_1c, beta_1s
    - mean inflow lambda_0
    - first-harmonic inflow lambda_1c, lambda_1s

Source-informed features
------------------------
- UH-60 main-rotor geometry and operating parameters from Lee (2021)
- 100 radial elements
- 1-degree azimuth integration
- -18 deg linear blade twist
- first-harmonic flapping representation
- linear inflow distribution
- Pitt-Peters matrix structure for first-harmonic inflow
- fuselage flat-plate drag relation from Eq. (2.32)

Important modeling choice
-------------------------
The dissertation does not provide every implementation detail required to
reconstruct the original TRAC load normalization, wake tables, blade
kinematics, control mapping, tail-rotor model, and empennage model.

A full three-state Pitt-Peters reconstruction using only the dissertation
produces ambiguous mean-inflow scaling in forward flight. To avoid hiding
that ambiguity behind fitted constants, this public example uses:

    mean inflow lambda_0:
        uniform momentum-theory closure

    harmonic inflow lambda_1c, lambda_1s:
        reduced steady Pitt-Peters structure

This hybrid closure is intentionally documented and should not be described
as the original TRAC inflow implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
import argparse

import numpy as np
from scipy.optimize import least_squares


LB_TO_N = 4.4482216152605
FT_TO_M = 0.3048
FT2_TO_M2 = FT_TO_M**2
KNOT_TO_MPS = 0.514444
KW_TO_HP = 1.34102209


@dataclass(frozen=True)
class UH60Parameters:
    """Parameters used by the public reconstruction."""

    gross_weight_lb: float = 16000.0
    density_altitude_ft: float = 5250.0

    blade_count: int = 4
    rotor_radius_ft: float = 26.83
    blade_chord_ft: float = 1.75
    rotor_speed_rad_s: float = 27.0
    first_airfoil_station_ft: float = 5.08
    linear_blade_twist_deg: float = -18.0
    solidity: float = 0.083
    lock_number: float = 5.11
    forward_mast_tilt_deg: float = 3.0

    # Explicit public-example assumptions.
    hinge_offset_ratio: float = 0.05
    lift_curve_slope_per_rad: float = 5.7
    section_cd: float = 0.011

    @property
    def weight_n(self) -> float:
        return self.gross_weight_lb * LB_TO_N

    @property
    def rotor_radius_m(self) -> float:
        return self.rotor_radius_ft * FT_TO_M

    @property
    def chord_m(self) -> float:
        return self.blade_chord_ft * FT_TO_M

    @property
    def first_airfoil_station_m(self) -> float:
        return self.first_airfoil_station_ft * FT_TO_M

    @property
    def rotor_area_m2(self) -> float:
        return np.pi * self.rotor_radius_m**2

    @property
    def tip_speed_mps(self) -> float:
        return self.rotor_speed_rad_s * self.rotor_radius_m


@dataclass(frozen=True)
class FlightCondition:
    """Straight-and-level comparison condition."""

    speed_kt: float
    gamma_deg: float = 0.0


@dataclass
class TrimResult:
    """Solved trim point and diagnostic outputs."""

    converged: bool

    collective_deg: float
    longitudinal_cyclic_deg: float

    beta0_deg: float
    beta1c_deg: float
    beta1s_deg: float

    lambda0: float
    lambda1c: float
    lambda1s: float

    fuselage_pitch_deg: float
    thrust_tilt_deg: float

    induced_power_kw: float
    profile_power_kw: float
    parasite_power_kw: float
    total_power_kw: float

    max_scaled_residual: float
    nfev: int


def isa_density(altitude_m: float) -> float:
    """ISA troposphere density approximation."""

    t0 = 288.15
    p0 = 101325.0
    lapse = 0.0065
    g = 9.80665
    r_air = 287.05287

    temperature = t0 - lapse * altitude_m
    pressure = p0 * (temperature / t0) ** (g / (r_air * lapse))

    return pressure / (r_air * temperature)


def comparison_pitch_deg(speed_kt: float) -> float:
    """
    Approximate fuselage-pitch schedule digitized from Lee (2021).

    It is used only for the dissertation-comparison sweep because the
    present reduced model does not contain the complete fuselage/tail
    pitching-moment balance needed to solve fuselage pitch independently.
    """

    speed = np.array(
        [0, 20, 40, 60, 80, 100, 110, 120, 140, 160],
        dtype=float,
    )

    pitch = np.array(
        [2.5, 2.4, 1.5, 0.8, 0.35, 0.2, 0.15, -0.5, -2.7, -5.8],
        dtype=float,
    )

    return float(np.interp(speed_kt, speed, pitch))


def comparison_alpha_f_deg(speed_kt: float) -> float:
    """
    Approximate fuselage flow-incidence schedule digitized from Fig. 2.13.

    It is used only inside the published UH-60 flat-plate drag relation.
    """

    speed = np.array(
        [0, 20, 40, 60, 80, 100, 110, 120, 140, 160],
        dtype=float,
    )

    alpha = np.array(
        [2.5, 2.3, 1.3, 0.55, 0.3, 0.2, 0.2, -1.0, -3.6, -6.3],
        dtype=float,
    )

    return float(np.interp(speed_kt, speed, alpha))


def fuselage_flat_plate_area_ft2(alpha_f_deg: float) -> float:
    """
    Lee (2021), Eq. (2.32).

    f(alpha_F) = 35.14 + 0.016 * (1.66 alpha_F)^2 [ft^2]
    """

    return 35.14 + 0.016 * (1.66 * alpha_f_deg) ** 2


class PublicTrimModel:
    """
    Source-informed reduced-order longitudinal UH-60 trim model.

    The model deliberately keeps lateral/yaw aircraft equilibrium outside
    the public example because reproducing it requires tail-rotor,
    empennage, wake-interference, and control-mapping data that are not
    fully specified in the dissertation.
    """

    def __init__(
        self,
        params: UH60Parameters | None = None,
        radial_elements: int = 100,
        azimuth_step_deg: float = 1.0,
    ) -> None:
        self.p = params or UH60Parameters()

        self.rho = isa_density(
            self.p.density_altitude_ft * FT_TO_M
        )

        self.root_ratio = max(
            self.p.first_airfoil_station_m / self.p.rotor_radius_m,
            self.p.hinge_offset_ratio,
        )

        radial = np.linspace(
            self.root_ratio,
            1.0,
            radial_elements,
        )

        azimuth = np.deg2rad(
            np.arange(
                0.0,
                360.0,
                azimuth_step_deg,
            )
        )

        self.RBAR, self.PSI = np.meshgrid(
            radial,
            azimuth,
            indexing="ij",
        )

        self.r = self.RBAR * self.p.rotor_radius_m

        self.dr = (
            (1.0 - self.root_ratio)
            * self.p.rotor_radius_m
            / (radial_elements - 1)
        )

        self.twist = np.deg2rad(
            self.p.linear_blade_twist_deg
        ) * (
            (self.RBAR - self.root_ratio)
            / (1.0 - self.root_ratio)
        )

        # Eq. (2.51): gamma = rho a c R^4 / I_b
        self.Ib = (
            self.rho
            * self.p.lift_curve_slope_per_rad
            * self.p.chord_m
            * self.p.rotor_radius_m**4
            / self.p.lock_number
        )

        # Eq. (2.57), uniform-blade hinge-offset approximation.
        e = self.p.hinge_offset_ratio

        self.nu_beta_sq = (
            1.0
            + 3.0 * e
            / (2.0 * (1.0 - e))
        )

    def fuselage_drag_n(
        self,
        condition: FlightCondition,
    ) -> float:
        """Dominant fuselage flat-plate drag term."""

        speed = condition.speed_kt * KNOT_TO_MPS

        alpha_f = comparison_alpha_f_deg(
            condition.speed_kt
        )

        area_m2 = (
            fuselage_flat_plate_area_ft2(alpha_f)
            * FT2_TO_M2
        )

        q = 0.5 * self.rho * speed**2

        return q * area_m2

    def blade_aerodynamics(
        self,
        x: np.ndarray,
        condition: FlightCondition,
    ) -> dict[str, object]:
        """
        Evaluate reduced blade-element loads.

        x =
        [theta0, theta1s, beta0, beta1c, beta1s,
         lambda0, lambda1c, lambda1s]
        """

        p = self.p

        theta0 = np.deg2rad(x[0])
        theta1s = np.deg2rad(x[1])

        beta0 = np.deg2rad(x[2])
        beta1c = np.deg2rad(x[3])
        beta1s = np.deg2rad(x[4])

        lambda0 = x[5]
        lambda1c = x[6]
        lambda1s = x[7]

        speed = condition.speed_kt * KNOT_TO_MPS

        beta = (
            beta0
            + beta1c * np.cos(self.PSI)
            + beta1s * np.sin(self.PSI)
        )

        d_beta_d_psi = (
            -beta1c * np.sin(self.PSI)
            + beta1s * np.cos(self.PSI)
        )

        d2_beta_d_psi2 = (
            -beta1c * np.cos(self.PSI)
            - beta1s * np.sin(self.PSI)
        )

        inflow = (
            lambda0
            + lambda1c
            * self.RBAR
            * np.cos(self.PSI)
            + lambda1s
            * self.RBAR
            * np.sin(self.PSI)
        )

        theta = (
            theta0
            + theta1s * np.sin(self.PSI)
            + self.twist
        )

        # Reduced blade-point velocities.
        # These preserve the dominant advancing/retreating asymmetry and
        # flap-velocity contribution without claiming to reproduce the full
        # transport-theorem implementation of Eqs. (2.33)-(2.43).
        u_t = (
            p.rotor_speed_rad_s * self.r
            + speed * np.sin(self.PSI)
        )

        u_p = (
            p.tip_speed_mps * inflow
            + p.rotor_speed_rad_s
            * (
                self.r
                - p.hinge_offset_ratio
                * p.rotor_radius_m
            )
            * d_beta_d_psi
        )

        # Source Eq. (2.48), linear incompressible blade-element lift.
        d_lift = (
            0.5
            * self.rho
            * p.lift_curve_slope_per_rad
            * p.chord_m
            * (
                theta * u_t**2
                - u_p * u_t
            )
            * self.dr
        )

        local_speed_sq = u_t**2 + u_p**2

        d_drag = (
            0.5
            * self.rho
            * local_speed_sq
            * p.chord_m
            * p.section_cd
            * self.dr
        )

        inflow_angle = np.arctan2(
            u_p,
            np.where(
                np.abs(u_t) > 1.0e-8,
                u_t,
                1.0e-8,
            ),
        )

        d_thrust = (
            d_lift * np.cos(inflow_angle)
            - d_drag * np.sin(inflow_angle)
        )

        thrust_n = float(
            p.blade_count
            * np.mean(
                np.sum(
                    d_thrust,
                    axis=0,
                )
            )
        )

        # Eq. (2.50): aerodynamic flap moment about the hinge.
        d_m_flap = (
            d_lift
            * (
                self.r
                - p.hinge_offset_ratio
                * p.rotor_radius_m
            )
        )

        m_flap_by_azimuth = np.sum(
            d_m_flap,
            axis=0,
        )

        # Reduced disk moment coefficients used only to drive the
        # first-harmonic inflow states.
        d_mx = (
            self.r
            * np.sin(self.PSI)
            * d_thrust
        )

        d_my = (
            self.r
            * np.cos(self.PSI)
            * d_thrust
        )

        mx_nm = float(
            p.blade_count
            * np.mean(
                np.sum(
                    d_mx,
                    axis=0,
                )
            )
        )

        my_nm = float(
            p.blade_count
            * np.mean(
                np.sum(
                    d_my,
                    axis=0,
                )
            )
        )

        profile_power_w = float(
            p.blade_count
            * np.mean(
                np.sum(
                    d_drag
                    * np.abs(u_t),
                    axis=0,
                )
            )
        )

        return {
            "beta": beta,
            "d2_beta_d_psi2": d2_beta_d_psi2,
            "thrust_n": thrust_n,
            "m_flap_by_azimuth": m_flap_by_azimuth,
            "mx_nm": mx_nm,
            "my_nm": my_nm,
            "profile_power_w": profile_power_w,
        }

    def harmonic_inflow_target(
        self,
        x: np.ndarray,
        condition: FlightCondition,
        aero: dict[str, object],
    ) -> np.ndarray:
        """
        Reduced steady Pitt-Peters first-harmonic target.

        The matrix structure follows Lee (2021), Eqs. (2.64)-(2.66).
        Only lambda_1c and lambda_1s are enforced from this target.

        lambda_0 is solved with a momentum-theory closure because the
        dissertation alone does not expose enough implementation detail
        to reconstruct the original mean-inflow/load normalization without
        ambiguity.
        """

        p = self.p
        lambda0 = float(x[5])

        speed = condition.speed_kt * KNOT_TO_MPS
        mu = speed / p.tip_speed_mps

        alpha = np.arctan2(
            max(lambda0, 1.0e-10),
            max(mu, 1.0e-10),
        )

        sin_alpha = np.sin(alpha)

        coupling = (
            15.0
            * np.pi
            / 64.0
            * np.sqrt(
                max(
                    0.0,
                    (1.0 - sin_alpha)
                    / (1.0 + sin_alpha),
                )
            )
        )

        # Reduced mass-flow parameter used for the harmonic closure.
        c_v = max(
            np.sqrt(
                mu**2
                + lambda0**2
            ),
            1.0e-8,
        )

        L = (
            1.0
            / c_v
            * np.array(
                [
                    [0.5, 0.0, coupling],
                    [
                        0.0,
                        -4.0
                        / (1.0 + sin_alpha),
                        0.0,
                    ],
                    [
                        coupling,
                        0.0,
                        4.0
                        / (1.0 + sin_alpha),
                    ],
                ],
                dtype=float,
            )
        )

        force_reference = (
            self.rho
            * p.rotor_area_m2
            * p.tip_speed_mps**2
        )

        moment_reference = (
            force_reference
            * p.rotor_radius_m
        )

        c_t = (
            float(aero["thrust_n"])
            / force_reference
        )

        c_my = (
            float(aero["my_nm"])
            / moment_reference
        )

        c_mx = (
            float(aero["mx_nm"])
            / moment_reference
        )

        return L @ np.array(
            [
                c_t,
                -c_my,
                c_mx,
            ],
            dtype=float,
        )

    def residuals(
        self,
        x: np.ndarray,
        condition: FlightCondition,
    ) -> np.ndarray:
        """Assemble the eight nonlinear trim residuals."""

        p = self.p

        aero = self.blade_aerodynamics(
            x,
            condition,
        )

        beta1c_deg = float(x[3])

        lambda0 = float(x[5])
        lambda1c = float(x[6])
        lambda1s = float(x[7])

        drag_n = self.fuselage_drag_n(
            condition
        )

        fuselage_pitch_deg = comparison_pitch_deg(
            condition.speed_kt
        )

        # Longitudinal-plane reduction of mast + TPP orientation.
        thrust_tilt = np.deg2rad(
            p.forward_mast_tilt_deg
            - fuselage_pitch_deg
            + beta1c_deg
        )

        thrust_n = float(
            aero["thrust_n"]
        )

        vertical_residual = (
            thrust_n
            * np.cos(thrust_tilt)
            - p.weight_n
        ) / p.weight_n

        longitudinal_residual = (
            thrust_n
            * np.sin(thrust_tilt)
            - drag_n
        ) / p.weight_n

        # Project Eq. (2.57) onto 0/rev and 1/rev harmonics.
        flap_point_residual = (
            aero["d2_beta_d_psi2"]
            + self.nu_beta_sq
            * aero["beta"]
            - aero["m_flap_by_azimuth"]
            / (
                self.Ib
                * p.rotor_speed_rad_s**2
            )
        )

        psi = self.PSI[0, :]

        flap_0 = float(
            np.mean(
                flap_point_residual
            )
        )

        flap_1c = float(
            2.0
            * np.mean(
                flap_point_residual
                * np.cos(psi)
            )
        )

        flap_1s = float(
            2.0
            * np.mean(
                flap_point_residual
                * np.sin(psi)
            )
        )

        speed = condition.speed_kt * KNOT_TO_MPS
        mu = speed / p.tip_speed_mps

        force_reference = (
            self.rho
            * p.rotor_area_m2
            * p.tip_speed_mps**2
        )

        c_t = thrust_n / force_reference

        # Uniform momentum-theory closure for mean inflow.
        lambda0_target = (
            c_t
            / (
                2.0
                * np.sqrt(
                    mu**2
                    + lambda0**2
                    + 1.0e-10
                )
            )
        )

        harmonic_target = self.harmonic_inflow_target(
            x,
            condition,
            aero,
        )

        return np.array(
            [
                vertical_residual,
                longitudinal_residual,
                flap_0,
                flap_1c,
                flap_1s,
                lambda0 - lambda0_target,
                lambda1c - harmonic_target[1],
                lambda1s - harmonic_target[2],
            ],
            dtype=float,
        )

    def solve(
        self,
        condition: FlightCondition,
        initial_guess: Iterable[float] | None = None,
        residual_tolerance: float = 1.0e-7,
    ) -> TrimResult:
        """Solve one public trim case."""

        if initial_guess is None:
            initial_guess = (
                22.0,
                0.5,
                2.5,
                -0.5,
                0.2,
                0.058,
                0.0,
                0.0,
            )

        lower = np.array(
            [
                5.0,
                -15.0,
                0.0,
                -10.0,
                -10.0,
                0.0,
                -0.10,
                -0.10,
            ],
            dtype=float,
        )

        upper = np.array(
            [
                35.0,
                15.0,
                10.0,
                10.0,
                10.0,
                0.20,
                0.10,
                0.10,
            ],
            dtype=float,
        )

        solution = least_squares(
            self.residuals,
            x0=np.asarray(
                tuple(initial_guess),
                dtype=float,
            ),
            bounds=(
                lower,
                upper,
            ),
            args=(
                condition,
            ),
            xtol=1.0e-11,
            ftol=1.0e-11,
            gtol=1.0e-11,
            max_nfev=2500,
            x_scale="jac",
        )

        residual = self.residuals(
            solution.x,
            condition,
        )

        aero = self.blade_aerodynamics(
            solution.x,
            condition,
        )

        drag_n = self.fuselage_drag_n(
            condition
        )

        (
            collective_deg,
            longitudinal_cyclic_deg,
            beta0_deg,
            beta1c_deg,
            beta1s_deg,
            lambda0,
            lambda1c,
            lambda1s,
        ) = solution.x

        fuselage_pitch_deg = comparison_pitch_deg(
            condition.speed_kt
        )

        thrust_tilt_deg = (
            self.p.forward_mast_tilt_deg
            - fuselage_pitch_deg
            + beta1c_deg
        )

        induced_velocity = (
            lambda0
            * self.p.tip_speed_mps
        )

        induced_power_w = (
            float(aero["thrust_n"])
            * induced_velocity
        )

        speed_mps = (
            condition.speed_kt
            * KNOT_TO_MPS
        )

        parasite_power_w = (
            drag_n
            * speed_mps
        )

        total_power_w = (
            induced_power_w
            + float(
                aero["profile_power_w"]
            )
            + parasite_power_w
        )

        max_residual = float(
            np.max(
                np.abs(
                    residual
                )
            )
        )

        return TrimResult(
            converged=bool(
                solution.success
                and max_residual
                <= residual_tolerance
            ),
            collective_deg=float(
                collective_deg
            ),
            longitudinal_cyclic_deg=float(
                longitudinal_cyclic_deg
            ),
            beta0_deg=float(
                beta0_deg
            ),
            beta1c_deg=float(
                beta1c_deg
            ),
            beta1s_deg=float(
                beta1s_deg
            ),
            lambda0=float(
                lambda0
            ),
            lambda1c=float(
                lambda1c
            ),
            lambda1s=float(
                lambda1s
            ),
            fuselage_pitch_deg=float(
                fuselage_pitch_deg
            ),
            thrust_tilt_deg=float(
                thrust_tilt_deg
            ),
            induced_power_kw=float(
                induced_power_w
                / 1000.0
            ),
            profile_power_kw=float(
                aero["profile_power_w"]
                / 1000.0
            ),
            parasite_power_kw=float(
                parasite_power_w
                / 1000.0
            ),
            total_power_kw=float(
                total_power_w
                / 1000.0
            ),
            max_scaled_residual=max_residual,
            nfev=int(
                solution.nfev
            ),
        )


def run_speed_sweep(
    model: PublicTrimModel,
    speeds: Iterable[float],
) -> list[
    tuple[
        FlightCondition,
        TrimResult,
    ]
]:
    """Continuation-style trim sweep."""

    guess = np.array(
        [
            22.0,
            0.5,
            2.5,
            -0.5,
            0.2,
            0.058,
            0.0,
            0.0,
        ],
        dtype=float,
    )

    results = []

    for speed in speeds:
        condition = FlightCondition(
            speed_kt=float(speed)
        )

        result = model.solve(
            condition,
            initial_guess=guess,
        )

        results.append(
            (
                condition,
                result,
            )
        )

        if result.converged:
            guess = np.array(
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

    return results


def print_speed_sweep(
    results: list[
        tuple[
            FlightCondition,
            TrimResult,
        ]
    ],
) -> None:
    """Print the sweep as a compact terminal table."""

    header = (
        f"{'V':>5} "
        f"{'theta0':>8} "
        f"{'theta1s':>8} "
        f"{'beta0':>8} "
        f"{'beta1c':>8} "
        f"{'beta1s':>8} "
        f"{'lam0':>9} "
        f"{'lam1c':>9} "
        f"{'lam1s':>9} "
        f"{'P[hp]':>9} "
        f"{'status':>8}"
    )

    print(header)
    print("-" * len(header))

    for condition, result in results:
        status = (
            "OK"
            if result.converged
            else "FAILED"
        )

        print(
            f"{condition.speed_kt:5.0f} "
            f"{result.collective_deg:8.3f} "
            f"{result.longitudinal_cyclic_deg:8.3f} "
            f"{result.beta0_deg:8.3f} "
            f"{result.beta1c_deg:8.3f} "
            f"{result.beta1s_deg:8.3f} "
            f"{result.lambda0:9.5f} "
            f"{result.lambda1c:9.5f} "
            f"{result.lambda1s:9.5f} "
            f"{result.total_power_kw * KW_TO_HP:9.1f} "
            f"{status:>8}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Public source-informed UH-60 trim example."
        )
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=60.0,
        help="Forward speed in knots.",
    )

    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Run 0-160 kt in 20-kt increments.",
    )

    args = parser.parse_args()

    model = PublicTrimModel()

    if args.sweep:
        results = run_speed_sweep(
            model,
            np.arange(
                0.0,
                161.0,
                20.0,
            ),
        )

        print_speed_sweep(
            results
        )
        return

    result = model.solve(
        FlightCondition(
            speed_kt=args.speed
        )
    )

    print(
        f"Converged: {result.converged}"
    )
    print(
        f"Speed: {args.speed:.1f} kt"
    )
    print(
        f"Collective: {result.collective_deg:.4f} deg"
    )
    print(
        "Longitudinal cyclic: "
        f"{result.longitudinal_cyclic_deg:.4f} deg"
    )
    print(
        "Flapping [beta0, beta1c, beta1s]: "
        f"[{result.beta0_deg:.4f}, "
        f"{result.beta1c_deg:.4f}, "
        f"{result.beta1s_deg:.4f}] deg"
    )
    print(
        "Inflow [lambda0, lambda1c, lambda1s]: "
        f"[{result.lambda0:.6f}, "
        f"{result.lambda1c:.6f}, "
        f"{result.lambda1s:.6f}]"
    )
    print(
        "Approximate power: "
        f"{result.total_power_kw * KW_TO_HP:.1f} hp"
    )
    print(
        "Max scaled residual: "
        f"{result.max_scaled_residual:.3e}"
    )


if __name__ == "__main__":
    main()
