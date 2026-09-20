# Harmony Aria

Harmony Aria is a compact, electrically powered, counterrotating coaxial-rotor personal aerial vehicle developed by Team Harmony for the Boeing-sponsored GoFly Prize.

Within this repository, Aria serves as an example of extending the modular Texas A&M University Rotorcraft Analysis Code (TRAC) beyond the baseline UH-60 configuration to a fundamentally different rotorcraft architecture.

The aircraft combines two independently controlled rigid coaxial rotors, electric propulsion, low-tip-speed rotor design, and electronically coupled swashplates. TRAC was used during the vehicle-development effort to model the aircraft flight dynamics, analyze stability, extract linearized models, and investigate closed-loop stabilization.

---

## 1. Role in TRAC

The UH-60 configuration provides the baseline implementation of the TRAC modeling framework.

Harmony Aria demonstrates a different use of the same modular architecture.

Rather than modifying the complete flight-dynamics formulation for a new aircraft, component models and aircraft-specific parameters can be replaced or reconfigured while retaining the common framework for:

- rigid-body equations of motion,
- component force and moment integration,
- rotor modeling,
- trim analysis,
- linearization,
- stability analysis,
- and control-system integration.

The Aria application therefore demonstrates the extensibility of TRAC from a conventional single-main-rotor helicopter to a counterrotating coaxial rotorcraft.

---

## 2. Aircraft Configuration

Aria was designed as a compact personal electric helicopter for the GoFly Prize.

The competition emphasized compactness, low acoustic signature, vertical-flight capability, endurance, and forward-flight performance. A counterrotating coaxial configuration was selected because it provides a large effective rotor disk area within a small aircraft footprint.

| Parameter | Aria configuration |
| --- | --- |
| Aircraft type | Personal electric helicopter |
| Rotor configuration | Counterrotating coaxial |
| Rotor location | Below operator |
| Number of rotors | 2 |
| Blades per rotor | 4 |
| Rotor diameter | 8.45 ft (2.58 m) |
| Prototype weight reported in paper | 520 lb (235.4 kg) |
| Rotor-design sizing weight | Approximately 750 lb (340 kg) |
| Power source | Electric |
| Battery capacity | 11 kWh |
| Rotor control | Independent collective and cyclic control |
| Flight-control mechanism | Dual independent swashplates with electronic coupling |

The 520 lb value corresponds to the full-scale prototype weight reported in the publication.

The rotor aerodynamic design was initially sized around an approximately 750 lb maximum takeoff weight. A separate component-weight breakdown in the publication gives 758.5 lb when a 200 lb payload is included. These values therefore represent different aircraft weight definitions and should not be treated as interchangeable.

---

## 3. Coaxial Rotor System

The coaxial configuration was selected primarily to obtain a large rotor disk area while remaining within the GoFly dimensional constraint.

Compared with a multirotor configuration occupying a similar footprint, the coaxial arrangement allows larger and slower-turning blades to be used. This reduces disk loading and blade-tip speed while providing both aerodynamic and acoustic benefits.

The upper and lower rotors rotate in opposite directions and are mechanically independent.

### Rotor Design Parameters

| Parameter | Value |
| --- | ---: |
| Rotor diameter | 8.45 ft (2.58 m) |
| Blades per rotor | 4 |
| Rotor solidity | 0.20 |
| Nominal rotor speed | 950 RPM |
| Hover tip Mach number | 0.37 |
| Disk loading | 13.22 lb/ft² (633 N/m²) |
| Design blade loading | 0.08 |
| Blade taper ratio | 2 |
| Blade twist | Approximately -9° overall |
| Airfoil | NASA RC(4)-10 |
| Rotor separation | Approximately 0.25 rotor radius |

A relatively high rotor solidity and low rotational speed were selected to reduce blade-tip Mach number.

The resulting hover tip Mach number of approximately 0.37 is substantially lower than that of many conventional full-scale helicopter rotors.

---

## 4. Rotor Aerodynamic Modeling

The rotor-development effort used multiple levels of aerodynamic analysis.

A blade-element-momentum-theory-based aerodynamic model was used together with the Pitt-Peters dynamic inflow model. Mutual aerodynamic interference between the upper and lower coaxial rotors was represented using a coaxial rotor interference model.

Higher-fidelity CFD calculations using CREATE-AV Helios were also used to evaluate the rotor design.

The analytical and CFD predictions were compared with experimental measurements obtained using a one-third-scale coaxial rotor test system.

The analytical model and CFD generally showed reasonable agreement with measured thrust and power, with most reported results within approximately 10% of the experimental measurements.

This multi-fidelity approach allowed computationally efficient models to be used during design exploration while higher-fidelity simulations and experiments were used for selected verification cases.

---

## 5. Acoustic-Oriented Rotor Design

Noise reduction was a major design objective for Aria.

Rotor RPM, blade loading, solidity, blade geometry, rotor separation, and airfoil selection were evaluated as coupled aerodynamic-acoustic design variables.

A low rotor-tip speed was particularly important because several major rotor-noise mechanisms strongly depend on blade velocity.

The rotor blades also incorporated a double-swept planform intended to reduce impulsive noise generated during blade-crossover events between the counterrotating rotors.

Rather than allowing the upper and lower rotor blades to interact simultaneously along a large portion of the span, the swept geometry spreads the interaction over time.

The acoustic analysis predicted a reduction of approximately 4.4 dBA from the double-swept blade design relative to the corresponding straight-blade configuration.

---

## 6. Propulsion System

Aria used an all-electric propulsion architecture with independent propulsion systems for the upper and lower rotors.

Each rotor was driven through its own electric motor, inverter, reduction gearbox, and cooling system.

The final full-scale configuration used:

| Component | Configuration |
| --- | --- |
| Motor | Emrax 228 axial-flux permanent-magnet motor |
| Number of motors | 2 |
| Transmission | Single-stage reduction gearbox |
| Reduction ratio | 3.5:1 |
| Cooling | Independent liquid-cooling system |
| Energy source | Lithium-ion battery |
| Battery capacity | Approximately 11 kWh |
| Battery energy density | Approximately 133 Wh/kg |

An earlier Emrax 208 motor configuration demonstrated high electrical efficiency but experienced excessive thermal loading during dynamometer testing.

The propulsion system was subsequently upgraded to the larger Emrax 228 motor together with active liquid cooling.

The drivetrain illustrates an important design trade-off in electric vertical flight: high motor efficiency does not eliminate the thermal-management requirements associated with sustained high-power operation.

---

## 7. Flight Controls

Aria used fixed-RPM rotors with blade-pitch control rather than rotor-speed modulation as the primary means of vehicle control.

The upper and lower rotors were equipped with mechanically independent swashplates that were electronically coordinated.

### Control Mechanisms

**Collective control**

Collective blade pitch changes the total rotor thrust and therefore primarily controls vertical motion.

During hover, the nominal collective settings reported for the vehicle were approximately:

| Rotor | Hover collective |
| --- | ---: |
| Upper rotor | 18.4° |
| Lower rotor | 21.4° |

The unequal collective values reflect the different aerodynamic environments experienced by the upper and lower rotors.

**Pitch and roll control**

Cyclic blade pitch generates asymmetric rotor loading.

Longitudinal cyclic produces pitching moment, while lateral cyclic produces rolling moment.

The maximum cyclic blade-pitch amplitude was approximately ±7°.

**Yaw control**

Unlike a conventional helicopter, Aria does not use a tail rotor.

Yaw control is produced by differential collective between the upper and lower rotors. Changing the torque balance between the two counterrotating rotors produces a net yawing moment.

A differential collective of approximately 2° provided the required yaw-control authority during hover and low-speed testing.

---

## 8. TRAC Flight-Dynamics Model

TRAC was used during the Aria development program to investigate aircraft stability and control.

In the TRAC implementation, Aria was represented as a rigid aircraft body with two independently controlled hingeless coaxial rotors.

This configuration differs substantially from the conventional UH-60 configuration and therefore provides an important demonstration of the modular aircraft-modeling architecture.

The general modeling process remained consistent with the TRAC framework:

1. Define the aircraft rigid-body properties.
2. Define the upper- and lower-rotor geometry and operating parameters.
3. Evaluate aerodynamic and inertial loads for each rotor.
4. Transform component loads into the aircraft body-fixed reference frame.
5. Sum forces and moments acting on the vehicle.
6. Determine equilibrium flight conditions.
7. Linearize the nonlinear model about the selected equilibrium.
8. Analyze the resulting aircraft dynamic modes.
9. Integrate a flight-control system with the linearized aircraft model.

The resulting model allowed the Aria configuration to be analyzed within the same general flight-dynamics architecture originally established using the UH-60.

---

## 9. Linearization and Stability Analysis

Two representative equilibrium conditions were examined using the TRAC Aria model:

| Flight condition | Analysis point |
| --- | --- |
| Hover | 0 kt |
| Forward flight | 46.7 kt |

The 46.7 kt condition represented the predicted maximum-endurance forward-flight condition used in the design analysis.

The nonlinear aircraft equations were linearized about each equilibrium condition and expressed in state-space form.

Eigenvalue analysis of the resulting linear models indicated unstable open-loop modes at both hover and forward flight.

A model-based Linear Quadratic Regulator (LQR) with nonzero set-point tracking was then applied.

With the LQR controller active, the relevant closed-loop poles moved into the stable left half of the complex plane in the reported analysis.

This analysis demonstrated how the TRAC framework could be used not only to model a new rotorcraft configuration but also to support control-system development.

---

## 10. Physical Flight-Control System

The model-based TRAC/LQR analysis should be distinguished from the flight-control system installed on the physical prototype.

The full-scale vehicle ultimately used the custom ELKA-R flight controller for stability augmentation.

ELKA-R used vehicle attitude and angular-rate feedback together with a proportional-integral-derivative control architecture.

The flight-control gains were adjusted during the progressive ground, gimbal, and tethered flight-test program.

Therefore:

- **TRAC + LQR** refers to the model-based stability and control analysis.
- **ELKA-R + PID** refers to the controller implemented on the physical experimental aircraft.

These two systems should not be described as the same controller.

---

## 11. Experimental Development and Flight Testing

Aria followed a progressive experimental-development process using both one-third-scale and full-scale vehicles.

The test sequence consisted primarily of:

1. gimbal testing,
2. tethered flight testing,
3. and free-flight testing.

### One-Third-Scale Vehicle

The subscale aircraft was used to evaluate rotor behavior, aircraft response, trimming, and flight-control characteristics before full-scale testing.

Wind-tunnel testing included forward-flow velocities up to approximately 29.2 kt.

The subscale vehicle also demonstrated outdoor forward flight at approximately 14.8 kt.

### Full-Scale Vehicle

The full-scale vehicle was initially evaluated using a dedicated gimbal test stand.

Tethered testing was subsequently used to evaluate stability, maneuverability, controller response, propulsion-system behavior, and acoustic performance before unrestricted flight.

During outdoor tethered testing, the measured overall sound pressure level was approximately:

**73 dBA at 50 ft (15.2 m)**

The result was close to the earlier acoustic predictions and substantially below the 87 dBA GoFly competition limit.

---

## 12. Hover Endurance Test

A full-scale hover endurance test was conducted from a fully charged battery until the minimum battery-cell cutoff voltage was reached.

The measured hover duration was approximately:

**8 minutes**

During this test:

- motor current increased from approximately 75 A to 85 A as battery voltage decreased,
- motor temperatures remained at or below approximately 65°C,
- and the two motor torques remained approximately balanced at 75 N·m each.

The test demonstrated that the propulsion and liquid-cooling systems could support the hover operating condition.

However, the approximately 8-minute endurance was substantially below the GoFly endurance objective and highlighted the energy-density limitation of the electric battery system used in the prototype.

---

## 13. Full-Scale Free Flight

The full-scale Aria achieved controlled outdoor free flight approximately two weeks before the GoFly final demonstration.

The vehicle flew in forward flight for approximately one minute at about 10 kt.

The flight subsequently ended in a roll excursion and ground impact that destroyed a significant portion of the aircraft.

The publication does not establish a definitive root cause.

Potential contributors discussed by the authors include:

- unexpected flight-controller actuator commands,
- retreating-blade stall associated with the low rotor RPM,
- and possible mechanical failure in the rotor pitch-link and swashplate system.

Because telemetry data were not successfully retained after the accident, the precise cause could not be conclusively determined.

The full-scale aircraft therefore did **not** experimentally demonstrate the greater-than-30-kt GoFly speed requirement.

Predicted capability and subscale forward-flight behavior should not be presented as full-scale flight-test verification.

---

## 14. Model Validation and Interpretation

Several different forms of evidence are associated with the Aria program and should be distinguished.

| Evidence | What it supports |
| --- | --- |
| Analytical rotor model | Rotor performance and design studies |
| Helios CFD | Higher-fidelity aerodynamic comparison |
| Subscale rotor tests | Rotor thrust and power comparison |
| Subscale flight tests | Configuration and controller feasibility |
| Full-scale gimbal/tether tests | Aircraft response and controller development |
| Full-scale acoustic measurements | Measured hover noise |
| Full-scale free flight | Demonstration of controlled flight at limited speed |
| TRAC stability analysis | Predicted local flight dynamics around selected equilibrium conditions |

The physical Aria flight tests should not automatically be interpreted as quantitative validation of the complete TRAC Aria flight-dynamics model.

The published work demonstrates that TRAC was used to model the aircraft, extract linearized dynamics, identify open-loop instability, and investigate LQR stabilization.

A detailed model-versus-flight-test system-identification comparison of the complete Aria flight-dynamics model was not reported.

This distinction is important when interpreting the level of model validation available for this configuration.

---

## 15. Significance for the TRAC Framework

Harmony Aria demonstrates an important capability of the TRAC architecture.

The original TRAC implementation was developed around a conventional UH-60 helicopter consisting of a main rotor, tail rotor, fuselage, horizontal tail, and vertical tail.

Aria required a fundamentally different configuration:

- two coaxial main rotors,
- no tail rotor,
- differential-collective yaw control,
- electric propulsion,
- independent swashplates,
- and substantially different mass and geometric characteristics.

Adapting TRAC to Aria demonstrates that the framework is not inherently restricted to the UH-60 configuration.

Instead, the aircraft can be represented as a collection of component models whose forces and moments are integrated within a common nonlinear flight-dynamics formulation.

This configuration-level reuse is one of the primary motivations for the modular TRAC architecture.

---

## 16. Scope and Limitations

The public documentation in this repository is intended to describe the modeling methodology and research application rather than reproduce the complete proprietary or research-development software environment.

The following limitations should therefore be considered:

- The complete original TRAC source code is not publicly released here.
- Not every aircraft parameter used during the original Aria simulations is currently included.
- UH-60 validation results cannot be transferred directly to the Aria configuration.
- The 46.7 kt TRAC condition was a modeled equilibrium point and not a demonstrated full-scale flight-test speed.
- Full-scale Aria testing did not reach the greater-than-30-kt GoFly speed objective.
- The physical flight-control system and the TRAC/LQR simulation represent different control implementations.
- The full-scale free-flight accident prevented additional dynamic-response and high-speed validation.

The Aria model should therefore be viewed primarily as a demonstrated application of the TRAC modeling architecture to a nonconventional coaxial rotorcraft configuration.

---

## 17. Relationship to Other Repository Documentation

General TRAC theory and analysis methods are documented separately:

- [`../../docs/framework-overview.md`](../../docs/framework-overview.md) — overall TRAC architecture
- [`../../docs/mathematical-model.md`](../../docs/mathematical-model.md) — governing aircraft equations
- [`../../docs/rotor-modeling.md`](../../docs/rotor-modeling.md) — rotor aerodynamic and dynamic modeling
- [`../../docs/trim-analysis.md`](../../docs/trim-analysis.md) — equilibrium and trim calculations
- [`../../docs/validation.md`](../../docs/validation.md) — model validation philosophy and results
- [`../UH-60/`](../UH-60/) — baseline UH-60 aircraft model

The UH-60 and Harmony Aria directories should be interpreted as aircraft-specific applications of the common TRAC framework.

---

## 18. References

1. Coleman, D., Halder, A., Saemi, F., Runco, C., Denton, H., Lee, B., Subramanian, V., Greenwood, E., Lakshminaryan, V., and Benedict, M.,  
   **“Development of ‘Aria,’ a Compact, Quiet Personal Electric Helicopter,”**  
   *Journal of the American Helicopter Society*, Vol. 68, 042011, 2023.  
   DOI: 10.4050/JAHS.68.042011

2. Lee, B.,  
   **“On the Complete Automation of Vertical Flight Aircraft Ship Landing,”**  
   Ph.D. Dissertation, Texas A&M University, 2021.

3. Coleman, D., Halder, A., Saemi, F., Runco, C., Denton, H., Lee, B., Subramanian, V., Greenwood, E., Lakshminaryan, V., and Benedict, M.,  
   **“Development of ‘Aria’, a Compact, Ultra-Quiet Personal Electric Helicopter,”**  
   Proceedings of the 77th Annual Forum and Technology Display of the Vertical Flight Society, 2021.
