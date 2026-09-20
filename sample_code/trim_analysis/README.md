# Trim Analysis

This directory contains the public executable trim example for the TRAC repository.

The objective is to expose the numerical workflow and the main rotorcraft modeling ideas documented in Lee (2021) without presenting this code as the original TRAC implementation.

## Recommended repository status

This version is intended to be the **public baseline sample** for `sample_code/trim_analysis/`.

Earlier exploratory reduced-order versions should not be committed alongside it unless they are explicitly placed in an `experimental/` or development-history directory.

---

## Model Scope

The example solves a source-informed longitudinal UH-60 trim problem.

The nonlinear unknown vector is:

```text
theta_0
theta_1s

beta_0
beta_1c
beta_1s

lambda_0
lambda_1c
lambda_1s
```

where:

- `theta_0` is main-rotor collective pitch,
- `theta_1s` is longitudinal cyclic pitch,
- `beta_0` is coning,
- `beta_1c` and `beta_1s` are first-harmonic flapping coefficients,
- `lambda_0` is mean inflow,
- `lambda_1c` and `lambda_1s` are first-harmonic inflow coefficients.

---

## Source-Informed Features

The following items follow the dissertation formulation or source-reported UH-60 configuration:

- four-blade main rotor,
- rotor radius of 26.83 ft,
- blade chord of 1.75 ft,
- rotor speed of 27 rad/s,
- first airfoil station at 5.08 ft,
- -18 degree linear blade twist,
- Lock number 5.11,
- 3 degree forward shaft tilt,
- 100 radial blade elements,
- 1-degree azimuth increment,
- linear incompressible blade-element lift structure,
- first-harmonic flapping representation,
- linear inflow distribution,
- Pitt-Peters matrix structure for first-harmonic inflow,
- UH-60 fuselage flat-plate drag relation.

---

## Explicit Public-Model Assumptions

The dissertation does not contain every implementation constant and lookup table required to reconstruct the complete original TRAC model.

The following values or reductions are therefore explicit assumptions in this example:

- 5% main-rotor hinge offset,
- lift-curve slope of 5.7 rad^-1,
- constant blade section drag coefficient of 0.011,
- reduced blade-point velocity equations,
- reduced disk pitching and rolling moment calculation,
- source-assisted fuselage pitch for the comparison sweep,
- omission of the complete tail-rotor force and moment model,
- omission of horizontal- and vertical-tail aerodynamics,
- omission of lead-lag trim states,
- omission of rotor-wake interference lookup tables.

These assumptions are visible in the code rather than hidden in fitted constants.

---

## Mean and Harmonic Inflow

Lee (2021) documents a three-state Pitt-Peters inflow model containing:

```text
lambda_0
lambda_1c
lambda_1s
```

A direct reconstruction of all three states from the dissertation alone is sensitive to details of rotor-load normalization, disk-moment conventions, wake geometry, and the complete coordinate transformation chain.

For this public example:

- `lambda_0` uses a uniform momentum-theory closure.
- `lambda_1c` and `lambda_1s` use a reduced steady Pitt-Peters matrix structure.

This hybrid treatment is intentional.

It produces a stable and transparent public example while avoiding the claim that undocumented implementation details have been reconstructed exactly.

---

## Installation

```bash
pip install -r requirements.txt
```

---

## Run a Single Case

```bash
python trim_example.py --speed 60
```

---

## Run the Forward-Speed Sweep

```bash
python trim_example.py --sweep
```

The default comparison sweep covers:

```text
0, 20, 40, 60, 80, 100, 120, 140, 160 kt
```

Continuation is used so that each converged point supplies the initial guess for the next speed.

---

## Compare With Dissertation Figures

```bash
python compare_with_dissertation.py
```

The script reads `reference_data.csv`.

The values in that file are approximate graph digitizations from the dissertation figures. They are **not** original numerical TRAC output and should not be treated as exact validation data.

The comparison is intended to answer:

- Does mean inflow follow the reported trend?
- Does the power curve show the expected low-speed/high-speed behavior?
- Does longitudinal first-harmonic flapping reproduce the main trend?
- Do first-harmonic inflow states behave plausibly?

---

## Current Comparison Summary

With the default parameters and the source-assisted forward-flight comparison conditions, the current public model gives approximately:

```text
lambda_0 mean absolute percentage difference: ~2.6%
power mean absolute percentage difference: ~6.9%
beta_1c mean absolute difference: ~0.40 deg
lambda_1s mean absolute difference: ~0.0029
```

These values are comparison diagnostics against approximate digitized curves, not formal validation metrics.

The coning coefficient `beta_0` remains systematically lower than the dissertation curve. This is retained rather than empirically tuned because the public model does not contain the complete original blade inertial, hinge, and airframe coupling implementation.

---

## Why This Version Is the Public Baseline

The sample is intentionally positioned between two undesirable extremes.

A very small algebraic model is easy to run but does not expose the rotor coupling that makes helicopter trim interesting.

A full apparent reconstruction would require filling missing implementation details with undocumented assumptions and could be mistaken for the original research code.

This version therefore includes enough physics to demonstrate:

```text
blade-element aerodynamics
        ↓
periodic rotor loading
        ↓
first-harmonic flapping
        ↓
mean + harmonic inflow
        ↓
aircraft force balance
        ↓
nonlinear trim solution
        ↓
speed-sweep continuation
        ↓
comparison diagnostics
```

while keeping all approximations visible.

---

## Files

```text
trim_analysis/
├── README.md
├── trim_example.py
├── compare_with_dissertation.py
├── reference_data.csv
├── requirements.txt
└── tests
    └── test_trim.py
```

`compare_with_dissertation.py` creates a local `comparison_results/` directory when it is run.

---

## Tests

Run:

```bash
pytest -q
```

The tests check:

- hover convergence,
- 60-knot convergence,
- full sweep convergence,
- decreasing mean inflow with forward speed,
- and recovery of power at high speed.

They are regression and behavior checks for the public example, not validation of TRAC.

---

## Limitations

This example must not be described as:

- the original TRAC source code,
- a complete UH-60 simulation,
- a six-degree-of-freedom validated flight model,
- or an exact reproduction of the dissertation figures.

The complete dissertation model integrates main rotor, tail rotor, fuselage, horizontal tail, vertical tail, rigid-body equations, blade motion, inflow dynamics, and component force/moment transformations.

This public example focuses on the main longitudinal trim mechanisms.

---

## Related Documentation

- [`../../docs/framework-overview.md`](../../docs/framework-overview.md)
- [`../../docs/mathematical-model.md`](../../docs/mathematical-model.md)
- [`../../docs/rotor-modeling.md`](../../docs/rotor-modeling.md)
- [`../../docs/trim-analysis.md`](../../docs/trim-analysis.md)
- [`../../docs/validation.md`](../../docs/validation.md)
- [`../../aircraft_models/UH-60/`](../../aircraft_models/UH-60/)

---

## Reference

Lee, B. (2021).  
*On the Complete Automation of Vertical Flight Aircraft Ship Landing*.  
Ph.D. dissertation, Texas A&M University.

Relevant sections:

- Section 2.3 — component modeling and integration
- Section 2.3.1 — fuselage aerodynamics
- Section 2.3.2.2 — main-rotor aerodynamic loads
- Section 2.3.2.4 — blade flapping dynamics
- Section 2.3.2.6 — Pitt-Peters inflow dynamics
- Section 2.4.1 — hover and steady forward-flight trim
- Appendix A — UH-60 configuration
