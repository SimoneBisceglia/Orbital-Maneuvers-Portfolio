# Orbital Maneuvers & Mission Analysis Portfolio

Developed GMAT and MATLAB simulations for advanced flight dynamics missions, focusing on trajectory optimization, Delta-V calculation, and perturbed environments.

## 📁 Project Structure: GMAT_Exercises

### 1. In-Plane Maneuvers
* **Hohmann vs. Bi-Elliptic Transfers:** Simulated radius change maneuvers.
* **Optimization:** Demonstrated that for large target-to-initial radius ratios, the Bi-Elliptic transfer is more fuel-efficient than Hohmann by exploiting lower velocities at higher apogees.
* **Validation:** All Delta-V budgets were calculated analytically in MATLAB and verified via GMAT numerical propagation.

### 2. Out-of-Plane Maneuvers & Perturbations
* **Inclination Changes (1-Impulse vs. 3-Impulse):** Evaluated plane change strategies, proving the energy efficiency of shifting the node to the apogee (3-Impulse) compared to a Single-Impulse burn.
* **General vs. Restricted Optimization:** Demonstrated that the General 3-Impulse method yields the absolute global minimum $\Delta V$ by optimizing thrust angles (alpha_1, alpha_2) across all maneuver nodes.
* **J2 Perturbation:** Modeled the Earth's oblateness (J2 effect) causing RAAN (Right Ascension of the Ascending Node) regression.
* **Target Compensation:** Implemented time-of-flight drift compensation to ensure precise target orbit insertion despite nodal drift.

### 3. Earth-Moon-Earth Mission
* **Lunar Transfer (Patched Conics):** Modeled the full trajectory architecture, computing escape (TLI) and capture (LOI) maneuvers for lunar orbit insertion and circularization.
* **Atmospheric Re-entry & Drag:** Simulated a high-energy return trajectory (11 km/s at 122 km interface), implementing atmospheric drag models to compute capture dynamics.
* **Numerical Targeting:** Solved complex convergence issues in GMAT using a 2-DOF strategy (varying Velocity and Normal burn components) to precisely hit the narrow          re-entry corridor.

---

## 🛠 Tools & Skills
* **Software:** Python (Orekit, NumPy, SciPy), MATLAB, GMAT (General Mission Analysis Tool).
* **Core Competencies:** Astrodynamics, Orbital Perturbations (J2, Drag), Trajectory Optimization, Numerical Targeting, Delta-V Budgeting.

> **Note:** Plot images and simulation scripts are located within their respective sub-folders under `GMAT_Exercises`.
