# Trim Analysis

This document summarizes the trim formulation and flight conditions presented in Section 2.4 of Lee (2021). The UH-60 is the baseline aircraft.

Trim establishes the control settings, aircraft attitude, rotor motion, and inflow required for a specified steady flight condition. The resulting operating point supports performance comparisons and local linearization.

This document describes the dissertation formulation. It does not establish a verified solver interface or provide executable reproduction of the reported results.

## 1. Definition of Trim

The dissertation defines trim through steady body-axis translational and angular motion. In terms of body-axis state derivatives:

```math
\dot{u}=\dot{v}=\dot{w}=0,
\qquad
\dot{p}=\dot{q}=\dot{r}=0.
```

These conditions do not require every state derivative to vanish. In particular, a steady turn can have nonzero body angular rates and a nonzero heading rate.

Zero body-axis velocity derivatives also do not imply zero inertial acceleration. The rotating-frame relation is:

```math
\mathbf{a}_{\mathrm{CG}}^{B}
=\dot{\mathbf{V}}^{B}
+\boldsymbol{\omega}^{B}\times\mathbf{V}^{B}.
```

Consequently, constant body-axis velocity components can describe a curved trajectory with centripetal acceleration.

Rotor trim likewise permits periodic blade motion. Constant first-harmonic coefficients do not imply that each blade remains at a fixed flap or lead-lag angle throughout a revolution.

## 2. Flight-Condition Parameters

Section 2.4 specifies flight conditions using trajectory speed, flight-path angle, and turn rate.

| Parameter | Meaning | Source convention |
| --- | --- | --- |
| $`V`$ | Speed along the trajectory | Nonnegative speed |
| $`\gamma`$ | Flight-path angle | Positive for climbing |
| $`\dot{\psi}`$ | Turn rate | Positive for a right turn |

| Flight condition | Defining conditions |
| --- | --- |
| Hover | Zero speed and zero turn rate; flight-path angle is geometrically undefined at zero speed |
| Straight and level flight | Nonzero speed, zero flight-path angle, zero turn rate |
| Straight climbing flight | Nonzero speed, positive flight-path angle, zero turn rate |
| Straight descending flight | Nonzero speed, negative flight-path angle, zero turn rate |
| Steady turning flight | Constant nonzero turn rate, with the remaining flight-condition constraints specified |

For bookkeeping, a hover input may set the flight-path-angle parameter to zero. This is a nominal input convention rather than a direction defined by a zero velocity vector.

Flight-path angle, fuselage pitch, and aerodynamic angle of attack are different quantities. They must not be substituted for one another without the relevant coordinate and velocity relationships.

## 3. Trim Unknowns

Dissertation Eq. (2.80) groups the unknowns into four partitions:

```math
\mathbf{x}_{\mathrm{trim}}
=\begin{bmatrix}
\mathbf{x}_C\\\mathbf{x}_F\\\mathbf{x}_R\\\mathbf{x}_I
\end{bmatrix}.
```

The notation is changed from the dissertation's capital X to distinguish the trim vector from the body-axis force component.

### 3.1 Control Angles

```math
\mathbf{x}_C
=\begin{bmatrix}
\theta_0&\theta_{1c}&\theta_{1s}&\theta_{TR}
\end{bmatrix}^{T}.
```

These represent collective, lateral cyclic, longitudinal cyclic, and tail-rotor control angles in the source convention. They are not automatically identical to pilot stick positions; comparisons with measured controls require the appropriate control mapping.

### 3.2 Fuselage Angles

```math
\mathbf{x}_F
=\begin{bmatrix}
\alpha_F&\beta_F&\phi_F&\theta_F
\end{bmatrix}^{T}.
```

The partition contains flow-incidence/sideslip variables and fuselage roll and pitch. Their relationships to trajectory constraints must follow the model's frame definitions. Figure-specific angle references should be checked before comparing reported values.

### 3.3 Rotor Motion Coefficients

```math
\mathbf{x}_R
=\begin{bmatrix}
\beta_0&\beta_{1c}&\beta_{1s}&
\zeta_0&\zeta_{1c}&\zeta_{1s}
\end{bmatrix}^{T}.
```

These coefficients describe the mean and first-harmonic flap and lead-lag motion:

```math
\beta(\psi_b)=\beta_0+\beta_{1c}\cos\psi_b+\beta_{1s}\sin\psi_b,
```

```math
\zeta(\psi_b)=\zeta_0+\zeta_{1c}\cos\psi_b+\zeta_{1s}\sin\psi_b.
```

Blade azimuth is denoted by $`\psi_b`$ to distinguish it from aircraft heading. At trim, the coefficients can remain constant while individual blade angles change periodically.

### 3.4 Inflow Coefficients

```math
\mathbf{x}_I
=\begin{bmatrix}
\lambda_0&\lambda_{1c}&\lambda_{1s}&\lambda_{TR}
\end{bmatrix}^{T}.
```

The main rotor uses uniform and first-harmonic Pitt-Peters inflow components. The tail rotor uses a uniform inflow ratio in the reported trim analysis.

These quantities are inflow ratios, not geometric angles. The four partitions contain 18 entries in total; flight-condition and kinematic constraints determine how the unknowns are related.

## 4. Residual Formulation

The aircraft model uses the coupled implicit system described in [Mathematical Model](mathematical-model.md):

```math
\mathbf{f}(\mathbf{y},\dot{\mathbf{y}},\boldsymbol{\delta},t)=\mathbf{0}.
```

A useful documentation-level representation of the trim problem is:

```math
\mathbf{R}_{\mathrm{trim}}
\left(\mathbf{x}_{\mathrm{trim}};V,\gamma,\dot{\psi}\right)
=\mathbf{0}.
```

This residual notation is a restatement for this document, not an additional numbered equation or a released function name from the dissertation.

The assembled problem must enforce compatible conditions for:

- Aircraft force and moment balance.
- Blade flap and lead-lag equilibrium under the adopted periodic representation.
- Steady inflow coefficients.
- Kinematics and the prescribed trajectory.
- The mapping between trim control angles and model control inputs.

The exact residual ordering, scaling, and elimination of dependent variables require implementation details beyond the 18-entry partition itself.

## 5. Hover and Straight Flight

For non-turning steady flight with constant Euler angles:

```math
p=q=r=0.
```

For straight and level flight:

```math
\gamma=0,\qquad\dot{\psi}=0.
```

Hover additionally has zero trajectory speed. Aerodynamic angles derived from ratios of velocity components require special treatment at hover because the translational velocity vector is zero.

The trim solution must balance all modeled components, including the main rotor, tail rotor, fuselage, and empennage. It should not impose zero fuselage pitch or roll merely because the trajectory is level.

### 5.1 Reported Speed Sweeps

| Comparison | Gross weight | Reported altitude | Speed range | Source |
| --- | --- | --- | --- | --- |
| Main-rotor power, case 1 | 16,360 lb | 3,670 ft density altitude | Hover to 160 knots | Fig. 2.11 |
| Main-rotor power, case 2 | 16,000 lb | 5,250 ft density altitude | Hover to 160 knots | Fig. 2.12 and accompanying text |
| Attitude, rotor motion, inflow, and controls | 16,000 lb | 5,250 ft | Hover to 160 knots | Figs. 2.13-2.21 |

The dissertation identifies Fig. 2.12 as a power comparison in its surrounding discussion despite the inconsistent printed caption. The topic above follows that discussion.

The source reports low-speed main-rotor power underprediction, especially below approximately 30 knots, and attributes this to the linear inflow assumption and rotor-wake interference. It reports better agreement above approximately 40 knots in the first power case and 50 knots in the second.

These observations describe the reported comparisons, not newly calculated error statistics. Detailed validation evidence belongs in [Validation](validation.md).

## 6. Climbing and Descending Flight

For the straight climb/descent cases, turn rate is zero and flight-path angle is varied while the specified speed is held fixed.

For a trajectory speed V, the geometric horizontal and upward velocity components are:

```math
V_h=V\cos\gamma,\qquad V_{\mathrm{up}}=V\sin\gamma.
```

In a North-East-Down frame, the corresponding down-axis velocity is:

```math
\dot{z}_G=-V\sin\gamma.
```

These are kinematic identities for the trajectory velocity, not separate aerodynamic assumptions. A reproduction must identify whether its input speed denotes trajectory speed, horizontal speed, or an airspeed quantity when wind is included.

The reported climb/descent sweep uses:

| Setting | Value |
| --- | --- |
| Gross weight | 16,000 lb |
| Altitude | 5,250 ft |
| Speed stated in Section 2.4.2 | 60 knots |
| Flight-path-angle range | -20 to +25 degrees |
| Compared outputs | Fuselage pitch and four pilot control inputs |
| Source figures | Figs. 2.22-2.26 |

The dissertation describes the 60-knot setting as forward flight speed, while Section 2.4 defines V along the trajectory. An executable reproduction should state the adopted speed interpretation explicitly.

The pitch comparison contains only five flight-test points according to the source. Agreement should therefore be described in terms of those available points rather than as dense validation of the entire sweep.

## 7. Steady Turning Flight

In a steady turn, heading changes at a constant nonzero rate. When roll and pitch are constant, the Euler-angle kinematics give:

```math
\dot{\phi}=0,\qquad\dot{\theta}=0,\qquad\dot{\psi}\ne0,
```

```math
p=-\dot{\psi}\sin\theta,
\qquad
q=\dot{\psi}\sin\phi\cos\theta,
\qquad
r=\dot{\psi}\cos\phi\cos\theta.
```

These identities follow from the 3-2-1 attitude kinematics. In general, the body yaw rate r is not equal to the heading rate. Setting all three body rates to zero would remove the steady-turn motion.

The reported turning analysis uses:

| Setting | Value |
| --- | --- |
| Gross weight | 16,000 lb |
| Altitude | 5,250 ft |
| Turn-rate range | -25 to +25 degrees per second |
| Turn convention | Positive for a right turn |
| Compared outputs | Roll/turn-rate relationship, pitch, and four control inputs |
| Source figures | Figs. 2.27-2.32 |

The measured data are organized by roll angle. The dissertation therefore first determines the roll-angle/turn-rate relationship and then compares attitude and control results against roll angle.

Section 2.4.3 does not explicitly restate the speed and flight-path angle in its prose. They should be confirmed from the full case definition before reproducing the turning sweep rather than automatically inherited from the preceding climb/descent case.

The coupled aircraft balance determines the required trim attitude. A simplified bank-angle/turn-rate relationship alone does not replace the rotorcraft trim problem.

## 8. Numerical Reproduction Notes

Section 2.4 defines the unknown partitions and presents results, but it does not specify a complete solver configuration, convergence tolerance, or residual scaling scheme.

The following is a proposed workflow for a future reproducible implementation, not a claim about the original source code:

1. Set the aircraft parameters, atmosphere, units, and flight condition.
2. Initialize the control, fuselage-angle, rotor-motion, and inflow unknowns.
3. Construct compatible velocities and angular rates from the trajectory constraints.
4. Evaluate component loads, periodic rotor balances, and inflow residuals.
5. Solve the assembled nonlinear trim equations with documented variable and residual scaling.
6. Check physical bounds and residual convergence before accepting the solution.
7. Save the accepted operating point and its diagnostic information.

For a speed or maneuver sweep, a converged nearby operating point can provide the next initial guess. This continuation strategy is an implementation suggestion; convergence at one point does not guarantee convergence or physical validity at the next.

### 8.1 Information to Record

| Category | Reproduction record |
| --- | --- |
| Aircraft | Configuration, gross weight, CG, inertia, rotor parameters, and control mapping |
| Atmosphere | Density or atmosphere model, altitude interpretation, and wind assumptions |
| Flight condition | Speed definition, flight-path angle, turn rate, and additional constraints |
| Numerical setup | Solver, initial guess, scaling, tolerances, and iteration limits |
| Trim result | All 18 partition entries and derived body velocities/angular rates |
| Diagnostics | Residual measures, convergence status, and any active bounds |
| Performance | Requested power, attitude, and control outputs with units |

An optimizer or nonlinear solver reporting success is insufficient if the resulting force, moment, rotor, or trajectory residuals remain unacceptable.

## 9. Connection to Linearization

Once an operating point has been established, Section 2.5 extracts the local model from derivatives of the coupled residual system:

```math
\mathbf{E}\Delta\dot{\mathbf{y}}
+\mathbf{F}\Delta\mathbf{y}
+\mathbf{G}\Delta\boldsymbol{\delta}=\mathbf{0}.
```

When the acceleration-coupling matrix is invertible:

```math
\mathbf{A}=-\mathbf{E}^{-1}\mathbf{F},
\qquad
\mathbf{B}=-\mathbf{E}^{-1}\mathbf{G}.
```

The trim vector and dynamic state vector are not interchangeable: the former includes control angles and harmonic coefficients, while the latter follows the chosen dynamic formulation. Their mapping must be explicit.

For steady-turn linearization, the perturbation description must account for the nominal heading evolution or use an appropriate reference representation. A turning trajectory should not be treated as a stationary earth-fixed state with every derivative zero.

## 10. Related Documents

- [Framework Overview](framework-overview.md)
- [Mathematical Model](mathematical-model.md)
- [Rotor Modeling](rotor-modeling.md)
- [Validation](validation.md)

## 11. Reference and Source Map

Lee, B. (2021). *On the Complete Automation of Vertical Flight Aircraft Ship Landing*. Ph.D. dissertation, Texas A&M University.

| Topic | Dissertation location |
| --- | --- |
| Trim definition, unknowns, and flight-condition parameters | Section 2.4, p. 53; Eq. (2.80) |
| Hover and steady forward flight | Section 2.4.1, pp. 54-60; Figs. 2.11-2.21 |
| Climbing and descending flight | Section 2.4.2, pp. 61-63; Figs. 2.22-2.26 |
| Steady turning flight | Section 2.4.3, pp. 63-66; Figs. 2.27-2.32 |
| Euler-angle/body-rate relations | Section 2.3.1, p. 36; Eq. (2.25) |
| Linearized model extraction | Section 2.5, pp. 67-68; Eqs. (2.81)-(2.85) |

Page numbers refer to the dissertation's printed pagination. Kinematic clarifications and the proposed numerical workflow are distinguished from the source's reported case settings and results.
