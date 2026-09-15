<p align="center">
  <img src="./assets/trac-overview.png"
       alt="TRAC Vertical Flight Aircraft Dynamics Modeling Framework"
       width="100%">
</p>

<h1 align="center">
TRAC
</h1>

<h3 align="center">
Texas A&M University Rotorcraft Analysis Code
</h3>

<p align="center">
<strong>
A Modular Vertical Flight Aircraft Dynamics Modeling and Simulation Framework
</strong>
</p>

---

# Overview

TRAC (Texas A&M University Rotorcraft Analysis Code) is a modular vertical flight aircraft dynamics modeling and simulation framework developed by Bochan Lee.

The framework was developed to provide a flexible computational environment for physics-based modeling, nonlinear flight dynamics analysis, trim computation, linear model extraction, and flight simulation of rotorcraft and emerging vertical-flight aircraft configurations.

Rather than being limited to a specific aircraft model, TRAC was designed with a modular architecture that enables individual aircraft components and configurations to be independently modeled, modified, and integrated into complete flight-dynamics systems.

The framework provides a foundation for research involving:

- conventional rotorcraft
- coaxial rotorcraft
- advanced vertical-flight aircraft
- autonomous flight systems

---

# Research Motivation

The increasing complexity of modern vertical-flight aircraft requires simulation frameworks capable of representing diverse aircraft configurations while maintaining sufficient physical fidelity for flight-dynamics analysis and control-system development.

TRAC was developed to bridge the gap between conceptual aircraft modeling and high-fidelity flight simulation by integrating:

- physics-based aircraft modeling
- nonlinear six-degree-of-freedom dynamics
- component-level aerodynamic representation
- control-oriented model extraction
- simulation-based validation

The framework enables investigation of aircraft behavior from early-stage configuration studies to advanced flight-control and autonomous-flight applications.

---

# Framework Architecture

TRAC represents a vertical flight aircraft as an integration of modular physical components.

Vertical Flight Aircraft Model

          |
          |
-----------------------
|          |          |

Fuselage Rotor Empennage
| | |
-----------------------
|
|
Flight Dynamics Solver
|
|
Simulation & Control Analysis


The modular architecture allows individual components to be modified, replaced, or extended without rebuilding the complete aircraft model.

This structure enables TRAC to support different aircraft configurations with consistent modeling and simulation processes.

---

# Core Capabilities

| Category | Capability |
|---|---|
| Aircraft Modeling | Modular component-based aircraft representation |
| Flight Dynamics | Nonlinear six-degree-of-freedom equations of motion |
| Rotor Modeling | Main rotor and tail rotor aerodynamic modeling |
| Aerodynamic Modeling | Component-level force and moment calculations |
| Trim Analysis | Hover, forward flight, climb/descent, and turning flight |
| Linearization | Flight-condition-dependent linear model extraction |
| Stability Analysis | Dynamic mode and eigenvalue analysis |
| Flight Simulation | Nonlinear aircraft response simulation |
| Control Integration | Interface for flight-control system development |
| Validation | Comparison with experimental flight-test data |

---

# Aircraft Modeling Capability

TRAC was developed as a general vertical-flight aircraft modeling framework rather than an aircraft-specific simulator.

Demonstrated aircraft configurations include:

- UH-60 conventional helicopter
- Harmony Aria coaxial rotorcraft

Future extensions include:

- multi-configuration rotorcraft
- variable-fidelity aircraft models
- novel vertical-flight aircraft configurations
- advanced VTOL and eVTOL concepts

---

# Demonstrated Aircraft Models

## UH-60 Helicopter

The UH-60 helicopter was selected as the baseline configuration for development and validation of the TRAC framework.

The model includes:

- fuselage aerodynamic representation
- main rotor model
- tail rotor model
- horizontal and vertical tail models
- nonlinear six-degree-of-freedom aircraft dynamics

The UH-60 model was validated through comparison with available U.S. Army flight-test data.

Validation demonstrated the capability of TRAC to reproduce key helicopter flight characteristics and provide a reliable foundation for flight-dynamics analysis and control-system development.

---

## Harmony Aria Coaxial Rotorcraft

TRAC was extended beyond conventional helicopter configurations through modeling of Harmony Aria, a compact coaxial electric rotorcraft.

The successful implementation demonstrated the extensibility of the framework toward different rotorcraft architectures without restructuring the complete simulation environment.

This capability provides a foundation for future studies involving diverse vertical-flight aircraft configurations.

---
