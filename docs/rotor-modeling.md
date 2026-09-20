# Rotor Modeling

This document summarizes the main-rotor and tail-rotor formulations in Chapter 2 of Lee (2021). The UH-60 is the baseline configuration.

The focus is the connection between blade motion, aerodynamic and inertial loads, induced inflow, and aircraft force and moment balance. This document describes the dissertation formulation rather than a verified interface to the public source code.

## 1. Model Scope

| Topic | Dissertation formulation |
| --- | --- |
| Main-rotor blades | Individually analyzed rigid blades |
| Blade motion | Flapping and lead-lag motion |
| Hinge arrangement | Coincident flap and lead-lag hinge location; spring and damper elements described in the model overview |
| Periodic motion | First-harmonic approximation for the blade-motion/load calculation described in Section 2.3.2 |
| Radial discretization | 100 elements per blade |
| Azimuth increment | 1 degree |
| Blade twist | Negative twist of 18 degrees included in local pitch calculation |
| Section aerodynamics | Linear incompressible lift model and constant drag coefficient |
| Main-rotor inflow | Pitt-Peters model with uniform and first-harmonic inflow components |
| Tail rotor | Simplified implementation of Bailey's closed-form approach |

These settings describe the dissertation baseline. They are not established defaults for every aircraft configuration or a claim that an executable implementation is included in this repository.

## 2. Coordinates and Notation

Rotor calculations use the body-fixed, blade non-rotating, blade rotating, blade flapped, and blade lagged frames defined in Section 2.2 of the dissertation.

A transformation $`\mathbf{T}_{AB}`$ maps vector components from frame B to frame A. Loads must be transformed into a common frame before addition.

To avoid overloading the aircraft attitude symbols, this document uses:

| Symbol | Meaning |
| --- | --- |
| $`\psi_b`$ | Blade azimuth |
| $`\beta`$ | Blade flapping angle |
| $`\zeta`$ | Blade lead-lag angle |
| $`\theta_b`$ | Local geometric blade pitch |
| $`\varphi_i`$ | Section inflow angle |
| $`R`$ | Main-rotor radius |
| $`\Omega`$ | Main-rotor angular speed |
| $`N_b`$ | Number of blades |
| $`eR`$ | Hinge offset distance, with dimensionless offset e |

Blade azimuth is distinct from aircraft yaw. Section inflow angle is distinct from aircraft roll. Rotor-frame directions and cyclic signs must follow the dissertation's conventions when implementing its component equations.

## 3. Blade Pitch and Section Aerodynamics

### 3.1 Geometric Pitch

Following dissertation Eq. (2.47), local pitch is written as:

```math
\theta_b(r,\psi_b)
=\theta_0+\theta_{1c}\cos\psi_b
+\theta_{1s}\sin\psi_b+\theta_{\mathrm{tw}}(r).
```

The dissertation identifies the cosine cyclic term with lateral cyclic and the sine cyclic term with longitudinal cyclic. The twist contribution depends on radial position.

Blade pitch angles are distinct from pilot stick positions. Their relationship requires the aircraft control mapping described in the underlying model.

### 3.2 Relative Velocity and Angle of Attack

The dissertation forms blade-relative flow using point velocity and induced velocity, then resolves the result into the blade frames. In the section convention of Fig. 2.8:

- Tangential velocity is positive for flow toward the leading edge.
- Radial velocity is positive outboard.
- Perpendicular velocity is positive downward.

Using the tangential and perpendicular components, the section angle of attack is:

```math
\alpha=\theta_b-\varphi_i,
\qquad
\varphi_i=\tan^{-1}\!\left(\frac{U_P}{U_T}\right).
```

This restates Eq. (2.46). Evaluation outside ordinary forward-flow conditions requires explicit quadrant and reverse-flow handling; the ratio form alone does not define that behavior.

### 3.3 Elemental Lift and Drag

For a blade element of span dr and chord c, Eq. (2.45) gives:

```math
dL=\frac{1}{2}\rho V_T^2 c_l c\,dr,
\qquad
dD=\frac{1}{2}\rho V_T^2 c_d c\,dr.
```

Here, $`V_T`$ denotes the local resultant speed used in the dissertation's section-load expression. Its linear aerodynamic assumption is:

```math
c_l=a\alpha,
\qquad c_d=\text{constant}.
```

With the small-inflow-angle and tangential-speed approximations used in Eq. (2.48), lift becomes:

```math
dL\approx\frac{1}{2}\rho a c
\left(\theta_b U_T^2-U_PU_T\right)dr.
```

The approximation is not a general stall, compressibility, or reverse-flow model. The assumptions should remain visible when interpreting results outside the linear aerodynamic range.

## 4. Inertial Loads

Blade inertial loads depend on the absolute acceleration of each blade point. Sections 2.3.2.1 and 2.2 connect that acceleration to aircraft motion, hub location, rotor rotation, flapping, and lead-lag motion.

In a single common frame, the position decomposition is:

```math
\mathbf{R}_P
=\mathbf{R}_{\mathrm{CG}}
+\mathbf{r}_{\mathrm{hub/CG}}
+\mathbf{r}_{P/\mathrm{hub}}.
```

Time derivatives require the transport theorem because the blade frames rotate. Differentiating rotating-frame components as though their basis vectors were fixed would omit rotational acceleration terms.

Writing blade mass per unit span as $`m_b(r)`$, a dimensional inertial-force contribution is:

```math
d\mathbf{F}_{I}=m_b(r)\mathbf{a}_{P}\,dr.
```

This is the mass-times-acceleration term. Its sign in a residual balance or a transmitted hub reaction depends on the chosen load convention; it is not automatically an additional applied aerodynamic force.

For a moment about the hinge, the position vector must run from the hinge to the element:

```math
d\mathbf{M}_{I,h}
=\mathbf{r}_{P/h}\times d\mathbf{F}_{I}.
```

All vectors in a cross product must be expressed in the same frame. Rotor inertial contributions must also be reconciled with the airframe inertia model to avoid double counting.

## 5. Flapping Dynamics

Section 2.3.2.4 develops the flapping balance from inertial, centrifugal, and aerodynamic moments about the flap hinge.

The first-harmonic blade-motion representation in Eq. (2.60) is:

```math
\beta(\psi_b)
=\beta_0+\beta_{1c}\cos\psi_b+\beta_{1s}\sin\psi_b.
```

The coefficients describe coning and the cosine and sine components of periodic flapping. Their physical orientation follows the adopted azimuth convention.

The blade flapping inertia about the offset hinge is:

```math
I_b=\int_{eR}^{R}m_b(r)(r-eR)^2\,dr.
```

For the uniform-blade idealization underlying the simplified frequency expression in Eq. (2.57):

```math
\nu_\beta^2=1+\frac{3e}{2(1-e)},
\qquad \omega_\beta=\nu_\beta\Omega.
```

Thus, a positive hinge offset raises this idealized flap frequency above one per revolution. The frequency expression does not by itself include every spring, damping, or coupling contribution of a more general rotor model.

The Lock number in Eq. (2.51) is:

```math
\gamma=\frac{\rho a c R^4}{I_b}.
```

It measures the relative influence of aerodynamic loading and blade inertia for the adopted blade idealization. Aerodynamic flapping moments also depend on inflow, pitch, blade motion, and aircraft angular rates.

## 6. Lead-Lag Dynamics

Section 2.3.2.5 develops the in-plane blade-motion balance about the lead-lag hinge using inertial, centrifugal, and aerodynamic drag moments.

The hinge inertia is:

```math
I_\zeta=\int_{eR}^{R}m_b(r)(r-eR)^2\,dr.
```

For the simplified uniform-blade centrifugal-restoring model of Eq. (2.62):

```math
\nu_\zeta^2=\frac{3e}{2(1-e)},
\qquad \omega_\zeta=\nu_\zeta\Omega.
```

This idealized lag frequency is lower than the corresponding flap frequency. The dissertation describes typical uncoupled rotating lag frequencies of approximately 0.2 to 0.3 times rotor speed for articulated rotors such as the UH-60.

These expressions summarize the uncoupled restoring behavior. They do not replace the coupled blade equations or establish complete damper and spring parameters for a released aircraft model.

## 7. Dynamic Inflow

The main-rotor inflow distribution in Eq. (2.63) is:

```math
\lambda(r,\psi_b)
=\lambda_0
+\lambda_{1c}\frac{r}{R}\cos\psi_b
+\lambda_{1s}\frac{r}{R}\sin\psi_b.
```

The uniform coefficient represents the mean contribution; the first-harmonic coefficients represent linear variation across the disk.

The dissertation uses the Pitt-Peters model, with the structure given in Eq. (2.64):

```math
\mathbf{M}_{\lambda}\dot{\boldsymbol{\lambda}}
+\mathbf{L}_{\lambda}^{-1}\boldsymbol{\lambda}
=\begin{bmatrix}C_T\\-C_{My}\\C_{Mx}\end{bmatrix},
\qquad
\boldsymbol{\lambda}
=\begin{bmatrix}\lambda_0\\\lambda_{1c}\\\lambda_{1s}\end{bmatrix}.
```

The subscripts on the matrices distinguish them from the rigid-body inertia and aircraft moments. The thrust and disk-moment coefficients couple rotor loading to inflow evolution.

This equation preserves the source's structural form. Matrix entries, inflow-state ordering, disk-axis signs, and time normalization must be used as a consistent set. The dot notation here does not independently establish whether an implementation differentiates with respect to dimensional or rotor-normalized time.

The detailed matrix and mass-flow parameter expressions are located in Eqs. (2.65)-(2.66). They are not reproduced here as an independently verified numerical implementation.

## 8. Blade Integration and Hub Loads

The blade calculation integrates distributed loads along the span and combines blade contributions after coordinate transformation.

To distinguish load density from an elemental force, let $`\mathbf{f}_n(r,\psi_b)`$ denote force per unit span for blade n:

```math
\mathbf{F}_n(\psi_b)=\int_{eR}^{R}\mathbf{f}_n(r,\psi_b)\,dr.
```

For instantaneous assembly into the non-rotating hub frame:

```math
\mathbf{F}_{\mathrm{hub}}^{NR}(t)
=\sum_{n=1}^{N_b}
\mathbf{T}_{NR,L_n}(t)\mathbf{F}_n^{L_n}(t).
```

Moments are combined in the same common frame and about the same hub reference point.

The dissertation also uses azimuth-integrated loads. For identical, equally spaced blades, a revolution-averaged force can be expressed as:

```math
\overline{\mathbf{F}}_{\mathrm{hub}}^{NR}
=\frac{N_b}{2\pi}\int_0^{2\pi}
\mathbf{T}_{NR,L}(\psi_b)\mathbf{F}_{\mathrm{blade}}^{L}(\psi_b)
\,d\psi_b.
```

The azimuth-dependent transformation belongs inside the averaging integral. An instantaneous blade sum and a revolution average describe different load quantities.

The hub loads are transferred to the aircraft body frame and center of gravity using the structure of Eq. (2.55):

```math
\mathbf{F}_{MR}^{B}
=\mathbf{T}_{B,NR}\mathbf{F}_{\mathrm{hub}}^{NR},
```

```math
\mathbf{M}_{MR,\mathrm{CG}}^{B}
=\mathbf{T}_{B,NR}\mathbf{M}_{\mathrm{hub}}^{NR}
+\mathbf{r}_{\mathrm{hub/CG}}^{B}\times\mathbf{F}_{MR}^{B}.
```

The moment-arm term is applied once. A moment already referenced to the aircraft center of gravity must not receive it again.

## 9. Tail-Rotor Model

Section 2.3.3 uses a simplified implementation of Bailey's closed-form solution to relate local flow to tail-rotor thrust, torque, and induced inflow.

The calculation accounts for:

1. Aircraft translational velocity and the body-rate contribution at the tail-rotor hub.
2. Interference flow associated with the main rotor and fuselage.
3. Transformation into tail-rotor axes, including its orientation.
4. Tail-rotor thrust, induced flow, and torque.
5. Transformation of tail-rotor loads and moment transfer to the aircraft center of gravity.

The source includes lookup-table interference functions dependent on wake skew and tip-path-plane tilt, together with a vertical-fin blockage factor in the thrust calculation.

The tail-rotor induced-velocity definition is:

```math
v_{i,TR}=\lambda_{TR}\Omega_{TR}R_{TR}.
```

The torque-coefficient relationship in Eq. (2.72) is:

```math
Q_{TR}=C_{Q,TR}\rho\pi\Omega_{TR}^{2}R_{TR}^{5}.
```

Torque magnitude must be combined with the rotor rotation and axis conventions to obtain the signed aircraft reaction moment. The dimensional thrust expression is not reproduced here as a verified implementation formula.

## 10. Connection to the Aircraft Model

Rotor and aircraft dynamics are coupled in both directions:

- Aircraft motion changes blade-point velocities and accelerations.
- Blade pitch and motion change aerodynamic loading.
- Rotor loading drives the dynamic inflow states.
- Inflow changes section angle of attack and loading.
- Hub forces and moments enter the aircraft residual equations.

Consequently, rotor states and inflow states participate in both trim and linearized model extraction. Rotor loading is not represented by a single constant thrust input.

See [Mathematical Model](mathematical-model.md) for the coupled residual system and [Trim Analysis](trim-analysis.md) for the flight-condition constraints.

## 11. Reference and Source Map

Lee, B. (2021). *On the Complete Automation of Vertical Flight Aircraft Ship Landing*. Ph.D. dissertation, Texas A&M University.

| Topic | Dissertation location |
| --- | --- |
| Rotor reference frames | Section 2.2, pp. 28-33 |
| Main-rotor assumptions and discretization | Section 2.3.2, p. 38 |
| Inertial loads | Section 2.3.2.1, pp. 38-40; Eqs. (2.33)-(2.38) |
| Aerodynamic loads and pitch | Section 2.3.2.2, pp. 40-44; Eqs. (2.39)-(2.52) |
| Hub loads | Section 2.3.2.3, pp. 44-45; Eqs. (2.53)-(2.55) |
| Flapping dynamics | Section 2.3.2.4, pp. 45-46; Eqs. (2.56)-(2.60) |
| Lead-lag dynamics | Section 2.3.2.5, pp. 47-48; Eqs. (2.61)-(2.62) |
| Dynamic inflow | Section 2.3.2.6, pp. 48-49; Eqs. (2.63)-(2.66) |
| Tail rotor | Section 2.3.3, pp. 49-51 |

Page numbers refer to the dissertation's printed pagination. Equations are selectively restated with explicit notation; this document is not a verbatim transcription of every source expression.
