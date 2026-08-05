# Orbital Maneuvers & Mission Analysis Portfolio

Developed GMAT, MATLAB, and Python (Orekit) simulations for advanced flight dynamics missions, focusing on trajectory optimization, Delta-V calculation and perturbation environments.

## 📁 Project Structure

### 1. GMAT & MATLAB Exercises (`GMAT_Exercises`)
*   **In-Plane Maneuvers:** 
    *   **Hohmann vs. Bi-Elliptic Transfers:** Simulated radius change maneuvers.
    *   **Optimization:** Demonstrated that for large target-to-initial radius ratios, the Bi-Elliptic transfer is more fuel-efficient than Hohmann by exploiting lower velocities at higher apogees.
    *   **Rendezvous Phasing:** Implemented targeted phasing maneuvers and co-elliptic/co-planar rendezvous sequences with target coordination.
    *   **Validation:** All Delta-V budgets were calculated analytically in MATLAB and verified via GMAT numerical propagation.
*   **Out-of-Plane Maneuvers & Perturbations:**
    *   **Inclination Changes (1-Impulse vs. 3-Impulse):** Evaluated plane change strategies, proving the energy efficiency of shifting the node to the apogee (3-Impulse) compared to a single-impulse burn.
    *   **General vs. Restricted Optimization:** Demonstrated that the General 3-Impulse method yields the absolute global minimum Delta-V by optimizing thrust angles across all maneuvers.
    *   **J2 Perturbation:** Modeled the Earth's oblateness (J2 effect) causing RAAN (Right Ascension of Ascending Node) regression.
    *   **Target Compensation:** Implemented time-of-flight drift compensation to ensure precise target orbit insertion despite nodal drift.
*   **Earth-Moon-Earth Mission:**
    *   **Lunar Transfer (Patched Conics):** Modeled the full trajectory architecture, computing escape (TLI) and capture (LOI) maneuvers for lunar orbit insertion and circularization.
    *   **Atmospheric Re-entry & Drag:** Simulated a high-energy return trajectory, implementing atmospheric drag models to compute capture dynamics.
    *   **Numerical Targeting:** Solved complex convergence issues in GMAT using a 2-DOF strategy to precisely hit the narrow re-entry corridor.
*   **Operational Orbits GMAT:**
    *   **Sun-Synchronous Orbit (SSO) Dawn-Dusk:** Simulated a 6 AM - 6 PM orbit, validating constant Local Solar Time (LST) and analyzing visual differences between inertial (ECI) and body-fixed (ECEF) ground tracks.
    *   **Molniya Constellation (HEO):** Designed a 3-satellite constellation at critical inclination (63.4°). Visualized apogee loops and continuous high-latitude coverage while mitigating J2-induced Argument of Perigee drift.
    *   **Geostationary Orbit (GEO):** Evaluated equatorial orbits matching Earth's rotation period for continuous regional coverage.

### 2. Python & Orekit High-Fidelity Simulations (`orekit/python`)
*   **In-Plane & Out-of-Plane Manoeuvres:** Python-based implementation of fundamental orbital transfers using Orekit wrappers.
*   **Earth-Moon Ephemeris:** Core scripts modeling the Earth-Moon system dynamics.
*   **High-Fidelity Perturbations & Propagators:** 
    *   Configured numerical propagators utilizing advanced integrators (Dormand-Prince 853).
    *   **LEO & SSO Analyses:** Modeled Sun-Synchronous Orbits (SSO) tailored for Earth observation and remote sensing missions, evaluating nodal regression and inclination coupling.
    *   **Molniya Orbits (HEO):** Analyzed highly elliptical orbits tailored for high-latitude telecommunications (focusing on orbital stability and long apogee dwell times).
    *   **Environmental Perturbations:** Integrated EGM96 gravity fields (harmonic expansions up to 10x10), Harris-Priester atmospheric drag models, Solar Radiation Pressure (SRP), and Luni-Solar Third Body gravitational forces to analyze real orbital decay and apsidal rotation (perigee argument drift).

---

## 🛠 Tools & Skills
*   **Software:** Python (Orekit, NumPy, SciPy), MATLAB, GMAT (General Mission Analysis Tool).
*   **Core Competencies:** Astrodinamica, Orbital Perturbations (J2, Drag, SRP, Third-Body), Trajectory Optimization, Numerical Targeting, Delta-V Budgeting, Orbital Rendezvous.

---
*Note: Plot images and simulation scripts are located within their respective sub-folders.*
