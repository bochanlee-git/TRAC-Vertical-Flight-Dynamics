# Linearization

This directory contains the public linearization examples for the TRAC repository.

The dissertation formulates the nonlinear system in implicit form:

```math
\mathbf{f}
\left(
\mathbf{y},
\dot{\mathbf{y}},
\mathbf{u},
t
\right)
=
\mathbf{0}.
```

At a trim point, first-order Taylor expansion gives:

```math
\mathbf{E}\Delta\dot{\mathbf{y}}
+
\mathbf{F}\Delta\mathbf{y}
+
\mathbf{G}\Delta\mathbf{u}
=
\mathbf{0},
```

with

```math
\mathbf{E}
=
\frac{\partial\mathbf{f}}
{\partial\dot{\mathbf{y}}},
\qquad
\mathbf{F}
=
\frac{\partial\mathbf{f}}
{\partial\mathbf{y}},
\qquad
\mathbf{G}
=
\frac{\partial\mathbf{f}}
{\partial\mathbf{u}}.
```

When `E` is nonsingular:

```math
\Delta\dot{\mathbf{y}}
=
\mathbf{A}\Delta\mathbf{y}
+
\mathbf{B}\Delta\mathbf{u},
```

where

```math
\mathbf{A}
=
-\mathbf{E}^{-1}\mathbf{F},
\qquad
\mathbf{B}
=
-\mathbf{E}^{-1}\mathbf{G}.
```

This is the workflow demonstrated here.

---

## Files

```text
linearization/
├── README.md
├── linearize_example.py
├── trim_sensitivity.py
├── requirements.txt
└── tests
    └── test_linearization.py
```

---

## 1. `linearize_example.py`

This file demonstrates the descriptor-form extraction itself.

It contains:

- a small nonlinear implicit longitudinal rotorcraft surrogate,
- central finite-difference Jacobian evaluation,
- extraction of `E`, `F`, and `G`,
- conversion to `A` and `B`,
- eigenvalue calculation,
- and a local linear-prediction check.

Run:

```bash
python linearize_example.py
```

Save matrices:

```bash
python linearize_example.py --save
```

Generated files are written to:

```text
linearization_results/
├── E.csv
├── F.csv
├── G.csv
├── A.csv
├── B.csv
└── eigenvalues.csv
```

### Important scope statement

The small dynamic system inside `linearize_example.py` is an **illustrative descriptor model**.

Its coefficients are not original UH-60 TRAC derivatives.

Its eigenvalues must therefore not be labeled as UH-60 modes or as reproduced dissertation results.

The purpose is to make the numerical procedure in Section 2.5 executable and inspectable.

---

## 2. `trim_sensitivity.py`

This file connects directly to the public model under:

```text
../trim_analysis/trim_example.py
```

It solves a trim point and evaluates the local Jacobian:

```math
\mathbf{J}_{trim}
=
\frac{\partial\mathbf{R}_{trim}}
{\partial\mathbf{x}_{trim}}.
```

Run the default 60-knot case:

```bash
python trim_sensitivity.py
```

Use another speed:

```bash
python trim_sensitivity.py --speed 30
```

Save the result:

```bash
python trim_sensitivity.py --speed 60 --save
```

The script reports:

- trim residual norm,
- Jacobian shape,
- numerical rank,
- singular values,
- and the 2-norm condition number.

This matrix is useful for trim-solver diagnostics and local parameter sensitivity.

It is **not** the flight-dynamics state matrix `A`.

---

## Why the Two Examples Are Separated

The dissertation states that the full TRAC state vector contains:

- nine airframe rigid-body states,
- induced-inflow states,
- and rotor-deflection states.

The control vector contains:

- collective,
- lateral cyclic,
- longitudinal cyclic,
- and pedal.

The public trim example in this repository does not reproduce that complete dynamic state vector.

Constructing a matrix and calling it the original UH-60 `A` matrix would therefore be misleading.

Instead, this directory separates:

```text
actual public trim-model sensitivity
                +
descriptor linearization method demonstration
```

This preserves a direct connection to the executable trim code while keeping the boundary between public reconstruction and original research software explicit.

---

## Numerical Differentiation

The examples use central finite differences:

```math
\frac{\partial f}{\partial x_i}
\approx
\frac{
f(x_i+h_i)-f(x_i-h_i)
}{
2h_i
}.
```

The variable-specific perturbation is:

```text
h_i = relative_step * max(1, abs(x_i))
```

The default relative step is:

```text
1e-6
```

A production implementation should check derivative convergence over multiple perturbation sizes because:

- steps that are too large increase truncation error,
- steps that are too small amplify floating-point cancellation,
- and variables with different units may require separate scaling.

---

## State-Space Interpretation

For the demonstration model:

```text
delta_y_dot = A delta_y + B delta_u
```

The eigenvalues of `A` describe only the local dynamics of the illustrative model.

For a complex pair:

```math
\lambda
=
\sigma
\pm
j\omega_d,
```

the real part `sigma` determines local exponential growth or decay and the imaginary part represents oscillation frequency.

No stability conclusion about the actual UH-60 should be drawn from the example eigenvalues.

---

## Direct Feedthrough

Lee (2021) writes the output transfer relation as:

```math
\mathbf{H}(s)
=
\mathbf{C}
(s\mathbf{I}-\mathbf{A})^{-1}
\mathbf{B}
+
\mathbf{D}.
```

The dissertation states that `D` is zero for the modeled aircraft because pilot inputs affect rotor force distributions and accelerations before appearing in the dynamic states and outputs.

This sample focuses on `E`, `F`, `G`, `A`, and `B`.

---

## Tests

Run:

```bash
pytest -q
```

The tests check:

- equilibrium residual,
- finite-difference recovery of the descriptor matrix,
- consistency of `A` and `B`,
- first-order local prediction behavior,
- convergence of the public 60-knot trim case,
- and rank of the trim residual Jacobian.

---

## Relationship to the Next Step

The natural next stage is:

```text
nonlinear model
      ↓
trim point
      ↓
linearization
      ↓
A / B matrices
      ↓
eigenvalue analysis
      ↓
feedback controller
      ↓
time-domain simulation
```

The repository's `simulation/` sample can use the `A` and `B` matrices produced by this directory to demonstrate open-loop and closed-loop propagation.

---

## Related Documentation

- [`../../docs/mathematical-model.md`](../../docs/mathematical-model.md)
- [`../../docs/trim-analysis.md`](../../docs/trim-analysis.md)
- [`../../docs/validation.md`](../../docs/validation.md)
- [`../trim_analysis/`](../trim_analysis/)
- [`../../aircraft_models/UH-60/`](../../aircraft_models/UH-60/)

---

## Reference

Lee, B. (2021).  
*On the Complete Automation of Vertical Flight Aircraft Ship Landing*.  
Ph.D. dissertation, Texas A&M University.

Relevant source locations:

- Section 2.1 — governing implicit nonlinear equations and state/control vectors
- Section 2.5 — extraction of the linearized model
- Eqs. (2.81)-(2.85) — Taylor expansion, Jacobians, state-space matrices, and transfer-function form
- Chapter 4 — examples of linearized UH-60 models used for LQR-based maneuver simulation
