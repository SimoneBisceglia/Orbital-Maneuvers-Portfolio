# Orbital Maneuvers & Mission Analysis Portfolio

Developed GMAT, MATLAB, and Python (Orekit) simulations for advanced flight dynamics missions, focusing on trajectory optimization, Delta-V calculation, and perturbation environments.

## 📁 Project Structure

### 1. GMAT & MATLAB Exercises (`GMAT_Exercises`)
*   **In-Plane Maneuvers:** 
    *   **Hohmann vs. Bi-Elliptic:** Simulated radius changes, proving Bi-Elliptic fuel efficiency for large radius ratios.
    *   **Rendezvous Phasing:** Implemented targeted co-elliptic/co-planar rendezvous sequences.
*   **Out-of-Plane Maneuvers & Perturbations:**
    *   **Inclination Changes:** Evaluated 1-Impulse vs. 3-Impulse strategies, proving the global Delta-V minimum of the General 3-Impulse method.
    *   **J2 Perturbation:** Modeled RAAN regression and implemented time-of-flight drift compensation for precise target orbit insertion.
*   **Earth-Moon-Earth Mission:**
    *   **Lunar Transfer:** Computed TLI/LOI patched-conics maneuvers for lunar capture and circularization.
    *   **Numerical Targeting & Re-entry:** Solved complex 2-DOF convergence issues in GMAT to precisely hit narrow atmospheric re-entry corridors.
*   **Operational Orbits GMAT:**
    *   **Sun-Synchronous Orbit (SSO):** Validated constant Local Solar Time (LST) and analyzed repeating ground tracks comparing ECI vs. ECEF visualization.
    *   **Molniya & GEO:** Designed a 3-satellite HEO constellation at critical inclination for continuous high-latitude coverage, and evaluated continuous regional coverage for GEO.
*   **Orbital Maintenance & Station Keeping (LEO & GEO):**
    *   **LEO Drag Make-Up (SMA Control):** Modeled rapid semi-major axis decay for a spacecraft with a low ballistic coefficient, implementing periodic tangential burns for step-like SMA recovery.
    *   **GEO Station Keeping:** Corrected luni-solar inclination drift via nodal impulses (North-South) and mitigated J22/SRP longitudinal drift via temporary SMA shifts within operational deadbands (East-West).
    *   **Methodology Note:** All Delta-V budgets were analytically sized in MATLAB and verified via GMAT. 3D orbit views were intentionally replaced with specialized 2D evolution plots (SMA, Inclination, Longitude vs. Time) to directly highlight secular variations and control boundaries.
    
### 2. Python & Orekit High-Fidelity Simulations (`orekit/python`)
*   **Core Mechanics:** Python-based implementation of orbital transfers and Earth-Moon ephemeris using Orekit wrappers.
*   **High-Fidelity Propagators:** Configured numerical propagators utilizing advanced integrators (Dormand-Prince 853).
*   **Environmental Perturbations:** Integrated EGM96 gravity fields, Harris-Priester atmospheric drag, Solar Radiation Pressure (SRP), and Luni-Solar Third Body forces to analyze orbital decay and apsidal rotation across LEO, SSO, and Molniya configurations.

---

## 🛠 Tools & Skills
*   **Software:** Python (Orekit, NumPy, SciPy), MATLAB, GMAT (General Mission Analysis Tool).
*   **Core Competencies:** Astrodynamics, Orbital Perturbations (J2, Drag, SRP, Third-Body), Trajectory Optimization, Numerical Targeting, Delta-V Budgeting, Orbital Rendezvous.

---
*Note: Plot images and simulation scripts are located within their respective sub-folders.*
