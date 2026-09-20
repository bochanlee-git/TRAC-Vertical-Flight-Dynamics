# Validation

This document summarizes the UH-60 flight-test comparisons reported in Section 2.4 of Lee (2021). It identifies the flight conditions, compared quantities, reported agreement, and limitations of the evidence.

The comparisons support assessment of the baseline model's steady-flight performance and trim behavior. They do not, by themselves, establish transient-response accuracy or validation of every rotorcraft configuration that TRAC could represent.

This is a source-based evidence summary. No new simulation, figure digitization, or numerical error calculation was performed for this document. The results below describe the dissertation model and do not certify a particular revision of the public repository.

## 1. Scope of the Evidence

| Evidence category | What it establishes |
| --- | --- |
| Main-rotor power compared with flight-test data | Performance agreement at the reported operating conditions |
| Fuselage attitude compared with flight-test data | Agreement of selected trim attitude outputs |
| Pilot control inputs compared with flight-test data | Agreement of required steady control settings |
| Predicted blade-motion and inflow coefficients | Internal model trends; not independent measurements unless a measured reference is supplied |
| A numerical trim solution | Satisfaction of the implemented equations, subject to convergence checks; not physical validation on its own |

The dissertation contains other vision, control, and flight-demonstration work. Those results should be documented under their respective systems rather than counted automatically as validation of the UH-60 flight-dynamics model.

## 2. Test-Condition Summary

| Case | Gross weight | Altitude stated in source | Sweep | Primary figures |
| --- | --- | --- | --- | --- |
| Main-rotor power, case 1 | 16,360 lb | 3,670 ft density altitude | Hover to 160 knots | 2.11 |
| Main-rotor power, case 2 | 16,000 lb | 5,250 ft density altitude | Hover to 160 knots | 2.12 |
| Forward-flight trim | 16,000 lb | 5,250 ft | Hover to 160 knots | 2.13-2.21 |
| Climb/descent trim | 16,000 lb | 5,250 ft | Flight-path angle from -20 to +25 degrees at a stated speed of 60 knots | 2.22-2.26 |
| Steady-turn trim | 16,000 lb | 5,250 ft | Turn rate from -25 to +25 degrees per second | 2.27-2.32 |

The sweep ranges describe the reported model cases. They do not imply that measured data exist at every operating point.

For climb/descent, Section 2.4.2 calls the setting forward flight speed, whereas Section 2.4 defines speed along the trajectory. A numerical reproduction should resolve this input interpretation explicitly.

The prose of Section 2.4.3 does not explicitly restate the turning-case speed and flight-path angle. Those settings should be confirmed before reproducing the case, rather than inherited silently from the preceding section.

## 3. Hover and Forward-Flight Comparisons

### 3.1 Main-Rotor Power

| Case | Source-reported result | Limitation |
| --- | --- | --- |
| 16,360 lb at 3,670 ft | Good agreement above approximately 40 knots | Power underpredicted below approximately 30 knots |
| 16,000 lb at 5,250 ft | Good agreement above approximately 50 knots | Similar low-speed power underprediction |

The dissertation attributes low-speed underprediction to the linear inflow assumption and the influence of rotor-wake interference. It notes that interference is stronger at low advance ratio and discusses a free-vortex wake model as a possible improvement.

This is a source-reported explanation and proposed improvement. The comparison does not demonstrate that a free-vortex wake implementation was added to, or tested within, the reported TRAC baseline.

The compared output is main-rotor power. It should not be relabeled as total engine or total aircraft power without reconciling tail-rotor power, transmission losses, and other relevant contributions.

### 3.2 Fuselage Attitude

| Output | Figure | Source-reported result |
| --- | --- | --- |
| Pitch angle | 2.14 | Good correlation over the reported forward-speed range |
| Roll angle | 2.15 | Similar overall trend, with underprediction of a few degrees at higher speeds |

Figure 2.13 presents additional predicted fuselage-angle trends. The explicit pitch and roll comparisons in Figs. 2.14-2.15 should be cited when discussing attitude validation.

The phrase "a few degrees" is the source's qualitative description, not a maximum error computed for this repository.

### 3.3 Pilot Controls

| Control | Figure |
| --- | --- |
| Collective stick | 2.18 |
| Longitudinal cyclic | 2.19 |
| Lateral cyclic | 2.20 |
| Pedal | 2.21 |

The dissertation reports close agreement with the available flight-test control data across the investigated forward-flight speeds.

When reproducing these plots, pilot stick position and rotor pitch angle must be distinguished. A matching control label is insufficient if the model and measurement use different gearing, normalization, units, or zero references.

### 3.4 Internal Rotor Quantities

Figure 2.16 presents predicted flap and lead-lag coefficients. Figure 2.17 presents predicted main-rotor and tail-rotor inflow ratios.

These plots help explain model behavior, but the accompanying discussion does not establish independent flight-test comparisons for those internal quantities. They should be described as predicted trim results rather than additional independently validated outputs.

## 4. Climbing and Descending Flight

The source compares fuselage pitch and four pilot control inputs at the reported 60-knot setting while varying flight-path angle from -20 to +25 degrees.

| Output | Figure | Evidence summary |
| --- | --- | --- |
| Fuselage pitch | 2.22 | Source describes reasonable agreement in trend and magnitude; only five flight-test points are noted |
| Collective stick | 2.23 | Compared with available flight-test data |
| Longitudinal cyclic | 2.24 | Compared with available flight-test data |
| Lateral cyclic | 2.25 | Compared with available flight-test data |
| Pedal | 2.26 | Compared with available flight-test data |

The source describes the control predictions as close to the available data. The sparse pitch measurements limit the strength of conclusions between measured points and near the ends of the simulated range.

This sweep assesses steady climb/descent trim. It does not establish the accuracy of transitions into a climb or descent, rapid vertical maneuvers, or descent regimes outside the reported comparisons.

## 5. Steady Turning Flight

The reported turn-rate sweep runs from -25 to +25 degrees per second. Positive turn rate denotes a right turn.

Because the flight-test data are organized by roll angle, the dissertation first investigates the relationship between turn rate and roll angle and then compares trim outputs against roll angle.

| Quantity | Figure | Role in the reported analysis |
| --- | --- | --- |
| Roll angle versus turn rate | 2.27 | Establishes the model relationship used to organize the turning case |
| Pitch angle versus roll angle | 2.28 | Presents turning trim attitude |
| Collective stick versus roll angle | 2.29 | Control comparison with flight-test data |
| Longitudinal cyclic versus roll angle | 2.30 | Control comparison with flight-test data |
| Lateral cyclic versus roll angle | 2.31 | Control comparison with flight-test data |
| Pedal versus roll angle | 2.32 | Control comparison with flight-test data |

The dissertation concludes that the control inputs are predicted closely while explicitly noting limited available flight-test data.

A steady-turn comparison is not a validation of turn-entry transients. Constant turn rate also does not imply zero body angular rates; see [Trim Analysis](trim-analysis.md) for the kinematic distinction.

## 6. Interpretation and Limitations

The reported comparisons show useful agreement for several steady UH-60 outputs, with identifiable low-speed power and high-speed roll discrepancies.

The evidence should be interpreted within the following limits:

- Agreement is specific to the documented configuration, atmosphere, and flight conditions.
- Low-speed power underprediction remains a reported limitation of the baseline.
- Climb/descent and turning measurements are limited.
- Internal rotor-state plots do not independently validate those states.
- Section 2.4 does not report RMSE, MAE, or a uniform numerical acceptance threshold for these comparisons.
- Steady-output agreement does not establish accurate stability derivatives, modal damping, frequency response, or transient dynamics.
- Generality of the framework does not establish equivalent validation for another aircraft configuration.

Numerical convergence and agreement with physical measurements answer different questions. Both should be recorded when a runnable reproduction is added.

## 7. Requirements for a Reproducible Comparison

The following records are proposed for future repository reproductions; they are not claimed to be available merely because the dissertation contains comparison figures.

| Record | Required content |
| --- | --- |
| Measurement provenance | Original report or dataset, relevant pages or tables, and conditions |
| Aircraft configuration | Gross weight, CG, rotor settings, control mapping, and applicable geometry |
| Atmosphere | Density altitude or other altitude definition, density calculation, and wind assumptions |
| Operating point | Speed definition, flight-path angle, turn rate, and any additional constraints |
| Numerical setup | Model revision, solver settings, initialization, residual scaling, and convergence results |
| Output definition | Units, coordinate axes, signs, reference points, and control normalization |
| Comparison data | Measured values, model predictions at matching conditions, and exclusions |
| Uncertainty | Measurement uncertainty when available, plus digitization uncertainty if figures are used |

If values must be digitized from figures, identify them as digitized estimates. Do not present them as original machine-readable flight-test records.

Interpolation must remain within a justified range and preserve the actual independent variable. In particular, the turning comparison uses roll angle, while the model sweep is described by turn rate.

## 8. Optional Quantitative Metrics

The metrics below are suggestions for future matched-data comparisons. No values have been calculated for this document, and these formulas are not presented as metrics reported by the dissertation.

For N matched points, define the prediction error as:

```math
e_i=y_i^{\mathrm{model}}-y_i^{\mathrm{test}}.
```

Useful summaries include mean bias, mean absolute error, and root-mean-square error:

```math
\mathrm{Bias}=\frac{1}{N}\sum_{i=1}^{N}e_i,
```

```math
\mathrm{MAE}=\frac{1}{N}\sum_{i=1}^{N}|e_i|,
\qquad
\mathrm{RMSE}=\sqrt{\frac{1}{N}\sum_{i=1}^{N}e_i^2}.
```

Report each metric with the output units and number of matched points. Inspect error versus flight condition as well as aggregate values so that low-speed or high-speed discrepancies remain visible.

Avoid percentage errors for angles or control quantities that cross zero unless a meaningful normalization scale is defined. Measurement uncertainty and sparse sampling should accompany any accuracy claim.

## 9. Source-Reading Notes

- Figure 2.12 is discussed as the second main-rotor power comparison, although its printed caption calls it a fuselage-angle plot. This document follows the surrounding power discussion.
- The second power discussion contains an inconsistent reference to Figure 3.1. The relevant local power comparisons are Figs. 2.11-2.12.
- The climb/descent discussion refers to a flight-speed range when describing controls, although that section varies flight-path angle at a stated speed. The comparison is identified here by its flight-path-angle sweep.

These notes clarify the source references. They do not establish errors in the underlying simulation code.

## 10. Related Documents

- [Framework Overview](framework-overview.md)
- [Mathematical Model](mathematical-model.md)
- [Rotor Modeling](rotor-modeling.md)
- [Trim Analysis](trim-analysis.md)

## 11. Reference and Source Map

Lee, B. (2021). *On the Complete Automation of Vertical Flight Aircraft Ship Landing*. Ph.D. dissertation, Texas A&M University.

| Topic | Dissertation location |
| --- | --- |
| Main-rotor power comparisons | Section 2.4.1, pp. 54-55; Figs. 2.11-2.12 |
| Fuselage attitude comparisons | Section 2.4.1, pp. 56-57; Figs. 2.13-2.15 |
| Predicted rotor motion and inflow | Section 2.4.1, pp. 57-58; Figs. 2.16-2.17 |
| Forward-flight control comparisons | Section 2.4.1, pp. 59-60; Figs. 2.18-2.21 |
| Climb/descent comparisons | Section 2.4.2, pp. 61-63; Figs. 2.22-2.26 |
| Steady-turn analysis and comparisons | Section 2.4.3, pp. 63-66; Figs. 2.27-2.32 |

Page numbers refer to the dissertation's printed pagination. Statements about agreement summarize the author's reported assessment rather than a new independent numerical evaluation.
