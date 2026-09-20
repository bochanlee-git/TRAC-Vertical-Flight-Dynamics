# Sample Code

This directory contains selected public examples illustrating the analysis workflow used in the Texas A&M University Rotorcraft Analysis Code (TRAC).

The examples are intended to demonstrate the numerical structure of rotorcraft flight-dynamics analysis, including:

- trim analysis
- linear model extraction
- stability analysis
- and time-domain simulation

They are not a release of the complete original TRAC research code.

---

## Directory Structure

```text
sample_code
├── trim_analysis
├── linearization
└── simulation
```

Each directory focuses on one stage of the flight-dynamics analysis workflow.

---

## 1. Trim Analysis

Directory:

```text
trim_analysis/
```

Trim analysis determines the aircraft state, control inputs, rotor states, and inflow states required to satisfy a prescribed steady-flight condition.

Representative conditions include:

- hover,
- steady forward flight,
- climb,
- descent,
- and coordinated turning flight.

The general workflow is:

```text
Aircraft configuration
        ↓
Flight condition
        ↓
Initial trim estimate
        ↓
Evaluate aircraft residual equations
        ↓
Update trim variables
        ↓
Check convergence
        ↓
Trim solution
```

Typical trim outputs include:

- aircraft attitude,
- collective input,
- cyclic inputs,
- pedal or yaw-control input,
- rotor flapping states,
- rotor lead-lag states,
- inflow states,
- and required rotor power.

See:

[`../docs/trim-analysis.md`](../docs/trim-analysis.md)

---

## 2. Linearization

Directory:

```text
linearization/
```

Once an equilibrium solution has been obtained, the nonlinear aircraft model can be locally linearized about the trim condition.

The resulting state-space representation can be used for:

- eigenvalue analysis,
- dynamic-mode identification,
- stability analysis,
- control-system design,
- and local response prediction.

The basic workflow is:

```text
Trim solution
      ↓
Perturb states and controls
      ↓
Evaluate nonlinear residuals
      ↓
Construct local Jacobian matrices
      ↓
Generate state-space model
      ↓
Analyze eigenvalues and modes
```

Each extracted model is local to its corresponding trim condition.

For example, a hover linear model should not automatically be interpreted as representing the aircraft dynamics at high forward speed.

See:

[`../docs/mathematical-model.md`](../docs/mathematical-model.md)

---

## 3. Simulation

Directory:

```text
simulation/
```

The simulation examples demonstrate how a flight-dynamics model can be propagated in time using prescribed control inputs or a feedback controller.

Possible applications include:

- response to control inputs,
- maneuver simulation,
- trajectory tracking,
- closed-loop stability evaluation,
- and flight-control research.

The general workflow is:

```text
Aircraft model
      +
Initial condition
      +
Control input
      ↓
Evaluate state derivatives
      ↓
Numerical integration
      ↓
Update aircraft states
      ↓
Repeat
```

TRAC was used in the original research to support control-oriented simulations using linearized aircraft models and Linear Quadratic Regulator (LQR) control.

---

## Relationship Between the Examples

The three directories represent a sequential flight-dynamics workflow:

```text
Nonlinear Aircraft Model
          ↓
     Trim Analysis
          ↓
   Equilibrium Point
          ↓
     Linearization
          ↓
   State-Space Model
          ↓
 Stability / Control
          ↓
      Simulation
```

A typical analysis therefore begins with a nonlinear aircraft model, determines an equilibrium condition, extracts a local linear model when required, and then uses the resulting model for stability or control analysis.

---

## Aircraft Configurations

The methodology represented by these examples is intended to remain independent of a particular aircraft configuration.

Aircraft-specific information is maintained separately under:

```text
aircraft_models/
├── UH-60/
└── Harmony_Aria/
```

The UH-60 serves as the baseline rotorcraft configuration documented in the TRAC research.

Harmony Aria demonstrates how the modular modeling framework can be adapted to a substantially different coaxial-rotor configuration.

---

## Scope of the Public Examples

The material in this directory should be interpreted as educational and research-portfolio examples.

Unless explicitly stated otherwise:

- the examples are simplified public implementations,
- they are not the complete original TRAC source code,
- they do not reproduce every aerodynamic or dynamic submodel,
- they may use representative or reduced-order data,
- and their results should not be presented as reproductions of dissertation results without separate verification.

The purpose is to expose the structure of the analysis rather than the complete research software implementation.

---

## Recommended Usage

The examples can be used to understand the progression from:

1. aircraft modeling,
2. equilibrium calculation,
3. local dynamic-model extraction,
4. stability analysis,
5. and closed-loop simulation.

For the theoretical background, refer to:

- [`../docs/framework-overview.md`](../docs/framework-overview.md)
- [`../docs/mathematical-model.md`](../docs/mathematical-model.md)
- [`../docs/rotor-modeling.md`](../docs/rotor-modeling.md)
- [`../docs/trim-analysis.md`](../docs/trim-analysis.md)
- [`../docs/validation.md`](../docs/validation.md)

For aircraft-specific information, refer to:

- [`../aircraft_models/UH-60/`](../aircraft_models/UH-60/)
- [`../aircraft_models/Harmony_Aria/`](../aircraft_models/Harmony_Aria/)

---

## Reference

Lee, B. (2021).  
*On the Complete Automation of Vertical Flight Aircraft Ship Landing*.  
Ph.D. dissertation, Texas A&M University.
