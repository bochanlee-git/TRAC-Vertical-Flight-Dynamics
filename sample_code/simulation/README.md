# Simulation

This directory completes the public sample workflow:

```text
trim analysis
      ↓
linearization
      ↓
state-space model
      ↓
open-loop simulation
      ↓
forced response
      ↓
LQR feedback simulation
```

The code uses the illustrative state-space model generated in:

```text
../linearization/linearize_example.py
```

It is **not** a UH-60 flight-trajectory reproduction.

---

## Source Relationship

Lee (2021) uses linearized UH-60 models extracted at trimmed flight conditions and couples them with an infinite-horizon continuous-time Linear Quadratic Regulator.

The dissertation defines the LQR objective as:

```math
J
=
\int_0^\infty
\left(
\mathbf{x}^T\mathbf{Q}\mathbf{x}
+
\Delta\mathbf{u}^T\mathbf{R}\Delta\mathbf{u}
\right)dt,
```

where the state error is measured relative to a desired target.

For set-point tracking, the control structure is:

```math
\Delta\mathbf{u}
=
\mathbf{u}_{ss}
-
\mathbf{K}
\left(
\mathbf{x}
-
\mathbf{x}_{ss}
\right).
```

The steady-state values satisfy:

```math
\mathbf{A}\mathbf{x}_{ss}
+
\mathbf{B}\mathbf{u}_{ss}
=
\mathbf{0}.
```

The dissertation then augments this with selected output constraints so non-zero reference states can be tracked.

The original UH-60 model has four pilot controls, so the dissertation notes that four non-zero reference states can be selected in that set-point calculation.

The public example here has only two controls, so it tracks two selected outputs.

---

## Files

```text
simulation/
├── README.md
├── simulation_example.py
├── plot_results.py
├── requirements.txt
└── tests
    └── test_simulation.py
```

---

## 1. Open-Loop Response

Run:

```bash
python simulation_example.py
```

The script first evaluates the zero-input response:

```math
\dot{\mathbf{x}}
=
\mathbf{A}\mathbf{x}.
```

A non-zero initial perturbation is used so the natural modes can be observed.

The state vector of the illustrative model is:

```text
u
w
q
theta
lambda
```

These names are retained from the public linearization example, but the corresponding coefficients are illustrative and must not be interpreted as identified UH-60 stability derivatives.

---

## 2. Forced Response

The script also applies small step-like perturbations in:

```text
collective
longitudinal_cyclic
```

and integrates:

```math
\dot{\mathbf{x}}
=
\mathbf{A}\mathbf{x}
+
\mathbf{B}\mathbf{u}.
```

This shows how the `B` matrix maps control perturbations into the state response.

---

## 3. LQR Design

The example constructs the continuous-time LQR gain by solving the algebraic Riccati equation.

The public weighting matrices are illustrative:

```text
Q = diag(2, 8, 4, 10, 2)
R = diag(1, 1)
```

They were not taken from the dissertation.

The dissertation explicitly notes that selection of `Q` and `R` affects:

- state-tracking priority,
- control-input priority,
- transient response,
- and required control effort.

It also states that realistic control behavior should be considered rather than optimizing set-point error alone.

That same interpretation should be used here.

---

## 4. Set-Point Tracking

Two outputs are selected:

```text
w
theta
```

The steady-state pair is obtained from:

```text
A x_ss + B u_ss = 0
C x_ss          = y_reference
```

and the controller uses:

```text
u = u_ss - K (x - x_ss)
```

This follows the set-point-tracking structure described in the dissertation.

The numerical target in this example is only a demonstration target.

It does not represent a documented UH-60 maneuver.

---

## 5. Generate CSV Output

Run:

```bash
python simulation_example.py --save
```

This produces:

```text
simulation_results/
├── open_loop.csv
├── forced_response.csv
├── lqr_tracking.csv
├── lqr_gain_K.csv
├── x_ss.csv
└── u_ss.csv
```

---

## 6. Generate Figures

Run:

```bash
python plot_results.py
```

The script creates:

```text
simulation_results/
├── open_loop_states.png
├── forced_response_states.png
├── lqr_tracking_outputs.png
└── lqr_controls.png
```

---

## Controllability

Before designing the LQR controller, the example forms the controllability matrix:

```math
\mathcal{C}
=
\begin{bmatrix}
\mathbf{B} &
\mathbf{AB} &
\mathbf{A}^2\mathbf{B} &
\cdots
\end{bmatrix}.
```

The illustrative model has full controllability rank.

This is a property of the public demonstration system, not a statement about a specific UH-60 linearization.

---

## Closed-Loop Stability

The LQR closed-loop system is:

```math
\dot{\mathbf{x}}
=
\left(
\mathbf{A}
-
\mathbf{B}\mathbf{K}
\right)
\mathbf{x}
+
\text{set-point terms}.
```

The example checks the eigenvalues of:

```text
A - B K
```

and confirms that they lie in the open left half-plane.

Again, those eigenvalues belong to the illustrative public model.

---

## Tests

Run:

```bash
pytest -q
```

The tests verify:

- full controllability rank,
- stable LQR closed-loop eigenvalues,
- accuracy of the steady-state set-point equations,
- reduction of tracking error,
- and finite simulation output.

These are software and numerical regression checks, not UH-60 validation tests.

---

## Scope and Limitations

The simulation sample should not be described as:

- a reproduction of the dissertation ship-landing trajectory,
- a validated UH-60 closed-loop model,
- the original TRAC LQR implementation,
- or the actual `Q`, `R`, or `K` matrices used in the research.

The dissertation used different linearized UH-60 models at different trimmed flight conditions.

For example, a 30-knot linearized model was used for portions of the ship-landing maneuver, while a 10-knot model was used for deceleration and landing.

The public example demonstrates the same **workflow concept** without inventing those unreleased system matrices.

---

## Repository Workflow

With this directory, the public examples now form:

```text
sample_code/
├── trim_analysis/
│   └── nonlinear equilibrium solution
│
├── linearization/
│   └── Jacobian / A-B extraction
│
└── simulation/
    └── open-loop / forced / LQR response
```

---

## Related Documentation

- [`../../docs/mathematical-model.md`](../../docs/mathematical-model.md)
- [`../../docs/trim-analysis.md`](../../docs/trim-analysis.md)
- [`../trim_analysis/`](../trim_analysis/)
- [`../linearization/`](../linearization/)
- [`../../aircraft_models/UH-60/`](../../aircraft_models/UH-60/)

---

## Reference

Lee, B. (2021).  
*On the Complete Automation of Vertical Flight Aircraft Ship Landing*.  
Ph.D. dissertation, Texas A&M University.

Relevant sections:

- Section 2.5 — linearized model extraction
- Section 4.1 — optimal control strategy
- Section 4.1.1 — LQR control design and set-point tracking
- Section 4.1.2 — ship-landing simulation results
