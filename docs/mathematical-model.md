# Mathematical Model

This document summarizes the mathematical formulation of the Texas A&M
Rotorcraft Analysis Code (TRAC) presented in Chapter 2 of Lee (2021).

The UH-60 helicopter serves as the baseline configuration. The formulation
combines airframe rigid-body motion, induced inflow, and rotor deflection
dynamics in a coupled nonlinear system.

This document describes the dissertation formulation. It does not define
a verified software interface or establish the implementation status of
the public repository.

## 1. Governing Equations

TRAC expresses the coupled equations in implicit residual form:

```math
\mathbf{f}
\left(
\mathbf{y},
\dot{\mathbf{y}},
\boldsymbol{\delta},
t
\right)
=
\mathbf{0}.
```

Here:

| Symbol | Description |
| --- | --- |
| $\mathbf{y}$ | System state vector |
| $\dot{\mathbf{y}}$ | Time derivative of the state vector |
| $\boldsymbol{\delta}$ | Pilot control input vector |
| $t$ | Time |

The notation $\boldsymbol{\delta}$ replaces the dissertation's control
vector $\mathbf{u}$ to distinguish control inputs from the forward
velocity component $u$.

Retaining the dependence on state derivatives allows the component
equations to represent acceleration coupling within the assembled system.

## 2. State and Control Vectors

### 2.1 State Partition

Following dissertation Eq. (2.2), the system state is partitioned as:

```math
\mathbf{y}
=
\begin{bmatrix}
\mathbf{y}_{F} \\
\mathbf{y}_{\lambda} \\
\mathbf{y}_{\mathrm{rotor}}
\end{bmatrix}.
```

| Partition | Description |
| --- | --- |
| $\mathbf{y}_{F}$ | Nine airframe rigid-body states |
| $\mathbf{y}_{\lambda}$ | Main-rotor and tail-rotor induced inflow coefficients |
| $\mathbf{y}_{\mathrm{rotor}}$ | Rotor deflection states for the blades |

The airframe state vector is:

```math
\mathbf{y}_{F}
=
\begin{bmatrix}
u & v & w & p & q & r & \phi & \theta & \psi
\end{bmatrix}^{T}.
```

| States | Description |
| --- | --- |
| $u,v,w$ | Body-axis translational velocity components |
| $p,q,r$ | Body-axis angular velocity components |
| $\phi,\theta,\psi$ | Roll, pitch, and yaw Euler angles |

Earth-fixed position may be propagated for trajectory simulation.
It is separate from the nine-state airframe partition above.

The total number of system states depends on the rotor and inflow
formulations included in the aircraft model.

### 2.2 Control Inputs

For the conventional helicopter configuration:

```math
\boldsymbol{\delta}
=
\begin{bmatrix}
\delta_0 &
\delta_{\mathrm{lat}} &
\delta_{\mathrm{lon}} &
\delta_{\mathrm{ped}}
\end{bmatrix}^{T}.
```

The inputs represent:

- Collective lever position.
- Lateral cyclic stick position.
- Longitudinal cyclic stick position.
- Pedal position.

These pilot inputs must be distinguished from blade pitch angles.
Their mapping depends on the aircraft control geometry and conventions.

## 3. Reference Frames and Conventions

The dissertation uses multiple reference frames to formulate component
motion and transfer loads into the aircraft body frame.

| Frame | Purpose |
| --- | --- |
| Earth-fixed, $G$ | Aircraft position and trajectory |
| Body-fixed, $B$ | Airframe motion and aircraft load balance |
| Blade non-rotating, $NR$ | Hub-centered reference incorporating shaft orientation |
| Blade rotating, $R$ | Blade azimuth-dependent motion |
| Blade flapped | Blade motion after flapping rotation |
| Blade lagged | Blade motion after lead-lag rotation |
| Tip-path-plane, $TPP$ | Rotor disk orientation under the adopted flapping representation |

The earth-fixed frame uses North-East-Down axes.

The body-fixed frame is located at the aircraft center of gravity, with
the axes directed forward, starboard, and downward.

Positive aircraft attitude directions are:

- Roll: right wing down.
- Pitch: nose up.
- Yaw: nose right.

In this document, a transformation $\mathbf{T}_{AB}$ maps vector
components from frame $B$ into frame $A$:

```math
\mathbf{a}^{A}
=
\mathbf{T}_{AB}\mathbf{a}^{B}.
```

Blade azimuth and aircraft yaw are distinct quantities, even though
the dissertation uses $\psi$ in both contexts.

### 3.1 Earth-to-Body Transformation

The aircraft attitude follows the 3-2-1 yaw-pitch-roll sequence:

```math
\mathbf{T}_{BG}
=
\mathbf{R}_{x}(\phi)
\mathbf{R}_{y}(\theta)
\mathbf{R}_{z}(\psi).
```

The passive coordinate-transformation matrices are:

```math
\mathbf{R}_{x}(\phi)
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos\phi & \sin\phi \\
0 & -\sin\phi & \cos\phi
\end{bmatrix},
```

```math
\mathbf{R}_{y}(\theta)
=
\begin{bmatrix}
\cos\theta & 0 & -\sin\theta \\
0 & 1 & 0 \\
\sin\theta & 0 & \cos\theta
\end{bmatrix},
```

```math
\mathbf{R}_{z}(\psi)
=
\begin{bmatrix}
\cos\psi & \sin\psi & 0 \\
-\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}.
```

The inverse transformation is:

```math
\mathbf{T}_{GB}
=
\mathbf{T}_{BG}^{T}
=
\mathbf{R}_{z}^{T}(\psi)
\mathbf{R}_{y}^{T}(\theta)
\mathbf{R}_{x}^{T}(\phi).
```

The order reverses when taking the transpose of a matrix product.

### 3.2 Euler-Angle Kinematics

Body angular rates and Euler-angle rates are related by:

```math
\begin{bmatrix}
\dot{\phi} \\
\dot{\theta} \\
\dot{\psi}
\end{bmatrix}
=
\begin{bmatrix}
1 & \sin\phi\tan\theta & \cos\phi\tan\theta \\
0 & \cos\phi & -\sin\phi \\
0 & \sin\phi/\cos\theta & \cos\phi/\cos\theta
\end{bmatrix}
\begin{bmatrix}
p \\ q \\ r
\end{bmatrix}.
```

Body angular rates are generally different from Euler-angle rates.

This representation is singular at pitch angles of
$\theta=\pm90^\circ$.

### 3.3 Position Kinematics

When earth-fixed position is included:

```math
\dot{\mathbf{r}}_{\mathrm{CG}}^{G}
=
\mathbf{T}_{GB}\mathbf{V}^{B},
\qquad
\mathbf{V}^{B}
=
\begin{bmatrix}
u \\ v \\ w
\end{bmatrix}.
```

Here, $\mathbf{V}^{B}$ is the inertial velocity of the aircraft center
of gravity expressed in body coordinates.

Aerodynamic calculations require air-relative velocity, which must
account for wind and the relevant induced-flow contributions.

## 4. Rigid-Body Equations

The equations below restate the rigid-body balance using explicit
force and inertia conventions.

They form part of the coupled aircraft model; rotor inertial loads
and rotor internal dynamics require consistent component accounting.

### 4.1 Translational Motion

Let the non-gravitational force expressed in body coordinates be:

```math
\mathbf{F}_{\mathrm{ng}}^{B}
=
\begin{bmatrix}
X \\ Y \\ Z
\end{bmatrix}.
```

The gravity vector in body coordinates is:

```math
\mathbf{g}^{B}
=
\mathbf{T}_{BG}
\begin{bmatrix}
0 \\ 0 \\ g
\end{bmatrix}
=
\begin{bmatrix}
-g\sin\theta \\
g\sin\phi\cos\theta \\
g\cos\phi\cos\theta
\end{bmatrix}.
```

With:

```math
\boldsymbol{\omega}^{B}
=
\begin{bmatrix}
p \\ q \\ r
\end{bmatrix},
```

the rigid-body translational balance is:

```math
m
\left(
\dot{\mathbf{V}}^{B}
+
\boldsymbol{\omega}^{B}
\times
\mathbf{V}^{B}
\right)
=
\mathbf{F}_{\mathrm{ng}}^{B}
+
m\mathbf{g}^{B}.
```

The component form is:

```math
\begin{aligned}
\dot{u}
&=
rv-qw+\frac{X}{m}-g\sin\theta, \\
\dot{v}
&=
pw-ru+\frac{Y}{m}+g\sin\phi\cos\theta, \\
\dot{w}
&=
qu-pv+\frac{Z}{m}+g\cos\phi\cos\theta.
\end{aligned}
```

Gravity is included explicitly and must not also be included in
$X$, $Y$, and $Z$.

### 4.2 Rotational Motion

For a rigid body with a constant body-frame inertia tensor:

```math
\mathbf{I}_{B}\dot{\boldsymbol{\omega}}^{B}
+
\boldsymbol{\omega}^{B}
\times
\left(
\mathbf{I}_{B}\boldsymbol{\omega}^{B}
\right)
=
\mathbf{M}_{\mathrm{CG}}^{B},
```

where:

```math
\mathbf{M}_{\mathrm{CG}}^{B}
=
\begin{bmatrix}
L \\ M \\ N
\end{bmatrix}.
```

Using products of inertia defined by integrals such as
$I_{xy}=\int xy\,dm$:

```math
\mathbf{I}_{B}
=
\begin{bmatrix}
I_{xx} & -I_{xy} & -I_{xz} \\
-I_{xy} & I_{yy} & -I_{yz} \\
-I_{xz} & -I_{yz} & I_{zz}
\end{bmatrix}.
```

The vector form makes the inertia and cross-product conventions
explicit.

In the coupled TRAC formulation, rotor inertial contributions must
be reconciled with the chosen airframe mass and inertia definitions.
The same inertia contribution must not be counted twice.

## 5. Component Load Integration

The baseline UH-60 model includes:

- Fuselage.
- Main rotor.
- Tail rotor.
- Horizontal tail.
- Vertical tail.

Component loads must be expressed in compatible coordinates and
referenced to a common point before they are assembled.

For component $c$, define:

- $\mathbf{F}_{c}^{c}$: force in the component frame.
- $\mathbf{M}_{c,\mathrm{ref}}^{c}$: moment about its reference point.
- $\mathbf{r}_{c}^{B}$: vector from the aircraft center of gravity
  to that reference point, expressed in body coordinates.

The force transformation is:

```math
\mathbf{F}_{c}^{B}
=
\mathbf{T}_{Bc}\mathbf{F}_{c}^{c}.
```

The moment transferred to the aircraft center of gravity is:

```math
\mathbf{M}_{c,\mathrm{CG}}^{B}
=
\mathbf{T}_{Bc}\mathbf{M}_{c,\mathrm{ref}}^{c}
+
\mathbf{r}_{c}^{B}\times\mathbf{F}_{c}^{B}.
```

Compatible applied loads are then summed:

```math
\mathbf{F}_{\mathrm{ng}}^{B}
=
\sum_c \mathbf{F}_{c}^{B},
\qquad
\mathbf{M}_{\mathrm{CG}}^{B}
=
\sum_c \mathbf{M}_{c,\mathrm{CG}}^{B}.
```

A moment already expressed about the aircraft center of gravity must
not receive the same moment-arm contribution again.

These expressions describe applied-load transfer. Component residual
equations must additionally preserve the sign and inertia conventions
used when assembling the implicit governing system.

### 5.1 Local Velocity

For a point fixed to the rigid airframe:

```math
\mathbf{V}_{P}^{B}
=
\mathbf{V}_{\mathrm{CG}}^{B}
+
\boldsymbol{\omega}^{B}\times\mathbf{r}_{P}^{B}.
```

If $\mathbf{V}_{\mathrm{air},P}^{B}$ denotes the local air velocity,
including the relevant wind and induced flow, the point velocity
relative to the air is:

```math
\mathbf{V}_{P/\mathrm{air}}^{B}
=
\mathbf{V}_{P}^{B}
-
\mathbf{V}_{\mathrm{air},P}^{B}.
```

The air velocity relative to the point has the opposite sign.
Aerodynamic angles and coefficient tables must use a consistent
relative-velocity convention.

Rotating and deflecting blade elements require additional
relative-motion terms.

## 6. Trim and Linearization

### 6.1 Trim Conditions

Trim is obtained by satisfying the coupled residual equations
together with constraints specifying the desired flight condition.

The dissertation considers:

- Hover and steady forward flight.
- Climbing and descending flight.
- Steady turning flight.

Trim does not require every state derivative to vanish.

For example, a steady turn can have constant body-frame velocity
components and nonzero body angular rates and heading rate.

The flight-condition constraints and trim solution procedure are
documented in [Trim Analysis](trim-analysis.md).

### 6.2 Linearized Dynamics

A first-order expansion of the residual equations about a selected
trim condition gives:

```math
\mathbf{E}\Delta\dot{\mathbf{y}}
+
\mathbf{F}\Delta\mathbf{y}
+
\mathbf{G}\Delta\boldsymbol{\delta}
=
\mathbf{0},
```

where:

```math
\mathbf{E}
=
\left.
\frac{\partial\mathbf{f}}
{\partial\dot{\mathbf{y}}}
\right|_{\mathrm{trim}},
```

```math
\mathbf{F}
=
\left.
\frac{\partial\mathbf{f}}
{\partial\mathbf{y}}
\right|_{\mathrm{trim}},
\qquad
\mathbf{G}
=
\left.
\frac{\partial\mathbf{f}}
{\partial\boldsymbol{\delta}}
\right|_{\mathrm{trim}}.
```

Here, $\mathbf{F}$ is a residual Jacobian, distinct from the
physical force vectors used above.

When $\mathbf{E}$ is invertible:

```math
\Delta\dot{\mathbf{y}}
=
\mathbf{A}\Delta\mathbf{y}
+
\mathbf{B}\Delta\boldsymbol{\delta},
```

with:

```math
\mathbf{A}
=
-\mathbf{E}^{-1}\mathbf{F},
\qquad
\mathbf{B}
=
-\mathbf{E}^{-1}\mathbf{G}.
```

For numerical computation, the equivalent linear systems are:

```math
\mathbf{E}\mathbf{A}=-\mathbf{F},
\qquad
\mathbf{E}\mathbf{B}=-\mathbf{G}.
```

Solving these systems avoids explicitly forming the inverse.

The resulting matrices describe local dynamics about the selected
trim condition.

### 6.3 Outputs and Transfer Functions

For an output perturbation vector $\Delta\mathbf{z}$:

```math
\Delta\mathbf{z}
=
\mathbf{C}\Delta\mathbf{y}
+
\mathbf{D}\Delta\boldsymbol{\delta}.
```

The corresponding transfer-function matrix is:

```math
\mathbf{H}(s)
=
\mathbf{C}
\left(
s\mathbf{I}-\mathbf{A}
\right)^{-1}
\mathbf{B}
+
\mathbf{D}.
```

The dissertation takes $\mathbf{D}=\mathbf{0}$ for the state-based
outputs discussed in Section 2.5.

This choice depends on the output definition; it should not be
assumed for every possible derived output.

## 7. Units and Notation

The dissertation reports quantities using units including feet,
pounds, knots, degrees, and horsepower.

Those reporting units do not establish a single internal software
unit system. Each released dataset or example should identify
its units explicitly.

Particular care is required when distinguishing:

- Mass from weight.
- Degrees from radians.
- Revolutions per minute from radians per second.
- Pilot control position from blade pitch angle.
- Inertial velocity from air-relative velocity.

The equations in this document normalize notation for clarity:

- The control vector is written as $\boldsymbol{\delta}$.
- Gravity is separated from non-gravitational force.
- Coordinate transformations use an explicit source-to-destination
  convention.
- The inverse attitude transformation follows
  $\mathbf{T}_{GB}=\mathbf{T}_{BG}^{T}$, consistent with dissertation
  Eq. (2.8).
- The roll matrix follows dissertation Eq. (2.4), including its
  nonzero third row.
- Rigid-body rotational balance is written in vector form.
- Local point velocity uses an explicitly defined CG-to-point vector.

These documentation conventions do not establish whether the original
source code contains any corresponding discrepancies.

## 8. Related Documents

- [Framework Overview](framework-overview.md)
- [Rotor Modeling](rotor-modeling.md)
- [Trim Analysis](trim-analysis.md)
- [Validation](validation.md)

## 9. Reference

Lee, B. (2021).
*On the Complete Automation of Vertical Flight Aircraft Ship Landing*.
Ph.D. dissertation, Texas A&M University.

Primary source locations:

| Topic | Dissertation location |
| --- | --- |
| Governing equations and state partitions | Section 2.1, pp. 25-26 |
| Coordinate systems and transformations | Section 2.2, pp. 26-33 |
| Component modeling and integration | Section 2.3, pp. 34-53 |
| Airframe states and rigid-body equations | Section 2.3.1, pp. 35-37 |
| Trim analysis | Section 2.4, pp. 53-66 |
| Linearized model extraction | Section 2.5, pp. 67-68 |

Page numbers refer to the dissertation's printed pagination.
