# ✈️ Project XB: UAV Longitudinal Stability Analysis

![Language](https://img.shields.io/badge/language-C++-blue.svg) ![Platform](https://img.shields.io/badge/platform-UAV%20Fixed%20Wing-orange.svg) ![Status](https://img.shields.io/badge/status-Analysis%20Complete-green.svg)

**An aerodynamic case study and numerical analysis tool for the "XB" Unmanned Aerial Vehicle concept.**

This project contains the flight dynamics solver developed to evaluate the **longitudinal static stability** and **trim conditions** of a custom-designed V-Tail UAV. It demonstrates the mathematical proof of why the initial design configuration resulted in static instability.

---

## 🖼️ The Concept: XB UAV

The XB was designed as a high-speed, high-altitude UAV platform. The design features a high-fineness ratio fuselage for drag reduction and a V-Tail configuration to minimize interference drag and structural weight.

<div align="center">

| Isometric View |
| :---: |
| ![XB Isometric](https://github.com/user-attachments/assets/ec749f99-bd18-4273-bc15-220baeb6ac07) |

*(Design and CAD modeling by Airwolf Dynamics team)*

</div>

## ⚙️ Design Configuration & Parameters

The solver is initialized with the specific geometric properties of the XB prototype, derived from the conceptual design report.

<div align="center">

| Component | Parameter | Symbol | Value | Unit |
| :--- | :--- | :---: | :---: | :---: |
| **Main Wing** | Chord Length | $\bar{c}$ | 0.45 | m |
| | Planform Area | $S$ | 0.8 | $m^2$ |
| | AC to CG Distance (Vertical) | $z$ | 0.1125 | m |
| **V-Tail** | Moment Arm (Longitudinal) | $l_t$ | 0.8 | m |
| | Vertical Offset from CG | $z_t$ | 0.06 | m |
| | Planform Area | $S_t$ | 0.16 | $m^2$ |
| | Dihedral Angle | $\Gamma$ | 20.6 | deg |
| **Propulsion** | Thrust Line Offset | $z_p$ | 0.02 | m |
| | Trim Thrust Required | $T_r$ | 25 | N |

</div>

## 🎯 The Engineering Problem

During the conceptual design phase, the primary challenge was to determine if the aircraft could achieve **Longitudinal Static Trim** and maintain **Static Stability** given the following constraints:

* **V-Tail Geometry:** Complex coupling of lift and drag moments due to dihedral angle ($\Gamma = 20.6^\circ$).
* **Wing Placement:** Large planform area positioned relative to the center of gravity (CG).
* **Aerodynamic Data:** Non-linear moment coefficients ($C_m$) obtained from XFLR5 analysis.

## 📐 Mathematical Model

The solver implements a **Newton-Raphson** numerical method to find the trim angle, handling the non-linear aerodynamic data robustly.

### 1. Governing Equations

The solver utilizes a component-based approach to determine the total pitching moment coefficient ($C_{m_{total}}$). The equilibrium condition is defined as:

$$C_{m_{total}} = C_{m,w} + C_{m,t} + C_{m,p} = 0$$

Where the individual contributions are derived as follows:

#### **A. Wing Moment Coefficient ($C_{m,w}$)**
The main wing contribution is calculated based on the Aerodynamic Center (AC) moment and the drag moment arm relative to the Center of Gravity (CG). Since the wing is mounted at zero incidence ($\alpha_w=0$):

$$C_{m,w} = C_{m,ac} + (C_{d,w})\frac{z}{\bar{c}}$$

*Where:*
* $C_{m,ac}$: Wing pitching moment at aerodynamic center (-0.17413).
* $z$: Vertical distance between AC and CG.
* $C_{d,w}$: Wing drag coefficient at cruise.

#### **B. V-Tail Moment Coefficient ($C_{m,t}$)**
Due to the V-Tail configuration with a dihedral angle ($\Gamma = 20.6^\circ$), aerodynamic forces are resolved into longitudinal and vertical components:

$$C_{m,t} = (C_{m,ac_t})\sin\Gamma - \left[ (a_t \alpha_t \cos\alpha_t)\cos\Gamma + (C_{d,t_0} + K_t(a_t \alpha_t)^2)\sin\alpha_t \right] \frac{l_t S_t}{\bar{c}S} - \left[ (a_t \alpha_t \sin\alpha_t)\cos\Gamma - (C_{d,t_0} + K_t(a_t \alpha_t)^2)\cos\alpha_t \right] \frac{z_t S_t}{\bar{c}S}$$

*Where:*
* $\Gamma$: Dihedral angle ($20.6^\circ$)
* $a_t \alpha_t$: Approximated Lift Coefficient ($C_{L,t}$)
* $C_{d,t_0} + K_t(C_{L,t})^2$: Drag Coefficient ($C_{D,t}$) using parabolic drag polar.
* $\frac{l_t S_t}{\bar{c}S}$ and $\frac{z_t S_t}{\bar{c}S}$: Longitudinal and Vertical Tail Volume Coefficients.

#### **C. Propulsion Moment Coefficient ($C_{m,p}$)**
The thrust line offset ($z_p$) from the Center of Gravity creates a constant pitching moment:

$$C_{m,p} = - \frac{2(z_p T_r)}{\rho V^2 S \bar{c}}$$

*Where:*
* $z_p$: Vertical distance between thrust line and CG.
* $T_r$: Thrust required for trim condition.

### 2. Numerical Solver
Instead of simple iteration, the tool uses a gradient-based approach to converge on the trim solution, even for unstable configurations:

$$\alpha_{new} = \alpha_{old} - \frac{f(\alpha)}{f'(\alpha)}$$

## 📊 Analysis Results (Updated with Modern Solver)

Initial historical analysis suggested the aircraft was unstable. However, the refactored **Newton-Raphson Solver** with proper data extrapolation revealed a critical insight:

* **Trim Angle:** $\approx -6.58^\circ$ (High, but feasible).
* **Stability Derivative ($C_{m_\alpha}$):** Calculated as **-0.02619 (Negative)**.
* **Conclusion:** The aircraft is **STATICALLY STABLE**.

## 🛠️ Usage

This tool is standalone and includes embedded aerodynamic data.

```bash
# Compile
g++ Trim_Analysis.cpp -o XB_Analyzer
./XB_Analyzer
```

## 📂 Project Structure
* Stability_Analyzer.cpp - The C++ source code with embedded physics engine.

* datat.txt - Raw aerodynamic coefficients exported from XFLR5.

* docs/ - Original design reports and parameter derivations.

  
##  👨‍💻 Author & Credits
**Kıvanç Apaydın** – Lead Flight Sciences & Software Development

**Airwolf Dynamics Team:**

Aerodynamic Analysis: Alperen Efe Köker, Emre Uçar

3D Modelling: Onur Açıkel
