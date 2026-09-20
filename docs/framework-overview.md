# Framework Overview

## 1. Purpose and Scope

TRAC (Texas A&M University Rotorcraft Analysis Code) is a modular rotorcraft flight-dynamics modeling and analysis framework developed by Bochan Lee. It combines component-level models to support aircraft performance prediction, trim analysis, linear model extraction, and control-oriented flight simulation.

The framework is intended to accommodate different rotorcraft configurations. The UH-60 serves as the baseline configuration documented and validated in the 2021 dissertation. Application to the Harmony Aria coaxial-rotor personal aerial vehicle is also reported.

This overview describes the framework documented in the dissertation; it does not imply that every research capability or aircraft dataset is included in this public repository.

## 2. Modeling Approach

TRAC represents an aircraft as an assembly of interacting component models. Each component evaluates its loads using the appropriate local flow conditions and reference frames. These loads are transformed and transferred to a common aircraft reference point and combined in the body-fixed frame.

For the baseline UH-60, the principal components are:

| Component | Modeling approach |
| --- | --- |
| Fuselage | Rigid-body dynamics and aerodynamic loads using empirical relationships and aerodynamic data. |
| Main rotor | Individual rigid-blade analysis with aerodynamic and inertial loads, flapping and lead-lag motion, and Pitt-Peters dynamic inflow. |
| Tail rotor | Simplified closed-form thrust and torque modeling with a one-state dynamic inflow model. |
| Horizontal tail | Stabilator aerodynamic loads using local flow conditions, empirical interference effects, and scheduled stabilator incidence. |
| Vertical tail | Fin aerodynamic loads using local flow conditions and aerodynamic lookup data. |

Coordinate transformations are implemented separately from component models. This organization supports component modification and alternative aircraft configurations while retaining the overall load-integration approach.

Modularity does not remove physical coupling: induced flow, local velocities, component locations, and load interactions remain part of the aircraft model.

The dissertation identifies HeliUM as the methodological starting point for TRAC, with aerodynamic modifications and additional analysis and control-simulation capabilities developed for this research.

## 3. Governing Model

The coupled aircraft model is expressed in implicit nonlinear form:

$$
\mathbf{f}(\mathbf{y},\dot{\mathbf{y}},\mathbf{u},t)=\mathbf{0}.
$$

The system state includes:

- Nine airframe rigid-body states: three translational velocities, three angular rates, and three Euler angles.
- Main-rotor and tail-rotor inflow states.
- Rotor blade deflection states.

For the conventional UH-60 configuration, the pilot-control inputs are collective, lateral cyclic, longitudinal cyclic, and pedal.

The formulation combines six-degree-of-freedom rigid-body dynamics with additional rotor and inflow dynamics. Consequently, six rigid-body degrees of freedom should not be interpreted as a six-state model.

Detailed equations, coordinate conventions, and state definitions belong in `docs/mathematical-model.md`.

## 4. Analysis Workflow

### 4.1 Aircraft and Flight-Condition Definition

The analysis begins with aircraft geometry, mass properties, component locations, aerodynamic information, and rotor parameters. The operating condition is specified using flight speed, flight-path angle, and turn rate, together with the relevant aircraft weight and atmospheric condition.

### 4.2 Component Loads and Integration

Local flow conditions are evaluated for each component. Aerodynamic and inertial contributions are calculated, transformed into compatible coordinates, and assembled at the aircraft reference point.

The main-rotor formulation uses individual blade loads and first-harmonic blade motion. The baseline inflow treatment uses the Pitt-Peters model rather than a free-wake or CFD-based wake calculation.

### 4.3 Trim Analysis

The trim solution determines the controls and internal model variables required to satisfy the prescribed steady-flight conditions and governing residual equations.

The dissertation groups the trim unknowns into:

- Control angles.
- Fuselage angles.
- Rotor flapping and lead-lag coefficients.
- Main-rotor and tail-rotor inflow coefficients.

Documented flight conditions include hover, steady forward flight, climb and descent, and steady coordinated turns. Outputs include aircraft attitude, control settings, rotor motion, inflow, and required power.

### 4.4 Linear Model Extraction

The nonlinear residual equations are linearized about a trim condition using a first-order Taylor expansion:

$$
\mathbf{E}\Delta\dot{\mathbf{y}}
+\mathbf{F}\Delta\mathbf{y}
+\mathbf{G}\Delta\mathbf{u}=\mathbf{0}.
$$

When the acceleration-coupling matrix is invertible, the resulting state-space model is:

$$
\Delta\dot{\mathbf{y}}
=\mathbf{A}\Delta\mathbf{y}
+\mathbf{B}\Delta\mathbf{u},
\qquad
\mathbf{A}=-\mathbf{E}^{-1}\mathbf{F},
\quad
\mathbf{B}=-\mathbf{E}^{-1}\mathbf{G}.
$$

Each extracted model represents local dynamics around its specified trim condition.

### 4.5 Control-Oriented Simulation

The dissertation uses extracted UH-60 models with linear quadratic regulator (LQR) control for ship-approach and landing simulations. Flight phases include descent, forward flight, coordinated turning, deceleration, and final vertical landing.

The simulations are performed in MATLAB and visualized in X-Plane. Corresponding trim-based models are used for different flight phases.

These UH-60 simulations assume perfect relative-position and orientation estimates. The separate vision-based quadrotor simulations and flight tests in the dissertation should not be described as flight testing of the UH-60 model.

## 5. Validation and Limitations

The dissertation compares UH-60 predictions with available U.S. Army flight-test data. Comparisons include main-rotor power, aircraft attitude, and control settings across selected steady-flight conditions. Rotor motion and inflow results are also presented.

The results demonstrate both useful agreement and identifiable limitations:

- Main-rotor power agreement is better at higher forward speeds in the reported comparisons.
- Power is underpredicted at low forward speeds, where the simplified inflow treatment does not fully capture wake-interference effects.
- Empirical aerodynamic information, including fuselage drag modeling, materially affects prediction accuracy.
- Agreement for selected UH-60 conditions does not establish accuracy for every flight regime or a different aircraft configuration.

The framework balances computational cost and modeling detail for flight-dynamics and control research. It should not be presented as a CFD solver or as having uniform fidelity throughout the flight envelope.

Detailed comparison conditions, figures, sources, and limitations belong in `docs/validation.md`.

## 6. Aircraft Configurations

| Configuration | Role documented in the dissertation |
| --- | --- |
| UH-60 | Baseline aircraft for component modeling, trim comparisons, linearization, and LQR-based ship-landing simulation. |
| Harmony Aria | Coaxial-rotor personal aerial vehicle application demonstrating reuse and adaptation of the modular modeling approach. |

UH-60 validation results should not be transferred to Harmony Aria as evidence of validation. Aircraft-specific assumptions, parameters, and supporting results should be documented separately.

## 7. Repository Documentation Map

The following paths define the intended organization of the public material. Files and examples are added progressively.

| Path | Intended content |
| --- | --- |
| `docs/mathematical-model.md` | Governing equations, coordinates, states, controls, and load integration. |
| `docs/rotor-modeling.md` | Rotor loads, blade motion, inflow, and modeling assumptions. |
| `docs/trim-analysis.md` | Flight-condition constraints, trim unknowns, and solution procedure. |
| `docs/validation.md` | Comparison methodology, reference data, results, and limitations. |
| `aircraft_models/UH-60/` | Publicly releasable UH-60 configuration information. |
| `aircraft_models/Harmony_Aria/` | Harmony Aria configuration and application information. |
| `sample_code/` | Selected trim, linearization, and simulation examples. |
| `results/` | Selected figures and analysis outputs with associated conditions. |
| `publications/` | Related publication information and permitted supporting material. |

This repository is a research portfolio containing selected documentation, results, and implementation examples. The complete TRAC source code is not part of the public release. Reproducibility should be stated for each released example rather than assumed for all dissertation results.

## 8. Source

Lee, B. (2021). *On the Complete Automation of Vertical Flight Aircraft Ship Landing*. Ph.D. dissertation, Texas A&M University.

Primary source sections:

- Section 1.2: Framework background and configuration extensibility.
- Chapter 2: Governing equations, component modeling, trim analysis, and linearization.
- Section 4.1: LQR-based UH-60 ship-approach and landing simulation.
- Section 6.1.1: Modeling conclusions, limitations, and Harmony Aria application.

This document summarizes the dissertation and distinguishes the original research framework from the selected material released in this repository.
