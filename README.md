# Astrodynamics, Flight Dynamics & GNC Portfolio

Developed GMAT, MATLAB, Simulink and Python (Orekit) simulations covering orbital mechanics, flight dynamics, perturbation modelling, ADCS, trajectory optimization and spacecraft navigation.

## 📁 Project Structure

### 1. GMAT & MATLAB Exercises (`GMAT_Exercises`)
*   **Orbital Transfers & Trajectory Design:** Modeled Hohmann vs. Bi-Elliptic transfers, targeted co-elliptic rendezvous sequences, and solved 2-DOF numerical targeting for Earth-Moon patched conics and narrow atmospheric re-entry corridors.
*   **Maneuver Optimization:** Evaluated 1-Impulse vs. 3-Impulse plane change strategies (finding global Delta-V minimums) and implemented J2 time-of-flight drift compensation.
*   **Operational Orbits & Station Keeping:** 
    *   Analyzed repeating ground tracks and constant LST for SSOs, and designed coverage configurations for HEO (Molniya) and GEO constellations.
    *   Simulated LEO drag make-up (SMA recovery) and GEO North-South/East-West station keeping within strict operational deadbands. 
    *   *Methodology:* Maneuvers analytically sized in MATLAB and verified in GMAT using 2D evolution plots to track secular variations.

### 2. Python & Orekit High-Fidelity Simulations (`orekit/python`)
*   **Astrodynamics & Transfer Optimization:**
    *   Implemented Python-based analytical validations for Earth-Moon Patched Conics (TLI/LOI Delta-V targeting).
    *   Demonstrated theoretical fuel efficiency limits for Bi-Elliptic vs. Hohmann transfers (r2/r1 > 11.94).
    *   Developed numerical optimizers (`scipy.optimize`) for Restricted and General 3-Impulse plane changes, extracting precise V, N, B burn components.
*   **High-Fidelity Perturbation Analysis:** 
    *   Configured `DormandPrince853` integrators with complex force models (10x10 Gravity Harmonics, Harris-Priester Drag, SRP, Luni-Solar Third Body).
    *   Ran 30-day propagations isolating specific orbital dynamics: LEO altitude decay, SSO nodal regression (J2 drift), GEO East-West tesseral drift, and the locked apsidal line of critical-inclination Molniya orbits (Frozen Orbits).
*   **SGP4 Propagation & Repeat Ground Track:** 
    *   Parsed real-world TLE data (CelesTrak) for analytical SGP4 propagation.
    *   Calculated exact repeating ground tracks (R revolutions in D days) compensating for J2 effects to extract longitudinal drift and equator crossing nodes, while evaluating track separation variations from the equator to high-latitude baselines to demonstrate physical footprint reduction driven by meridian convergence.
*   **Navigation & Orbit Determination (EKF):**
    *   Generated high-fidelity truth trajectories and simulated black-box GNSS SPP-level PVT measurements (Gaussian noise injection).
    *   Implemented an On-Board Computer (OBC) navigation model using Orekit's `KalmanEstimator` to fuse noisy sensor data with the dynamic model, demonstrating robust filter convergence and significant 3D RMSE reduction over raw GNSS data, despite highly degraded initial states.
### 3. ADCS & GNC Slew Maneuver Project (`ADCS_GNC_Project`)
*   **Ideal Kinematics & Baseline:**
    *   Implemented a 3-DOF attitude dynamics and PD control baseline in Simulink using a quintic S-curve guidance profile for an off-nadir slew  within a 120-second window.
    *   Documentation includes the top-level architecture schema illustrating the ideal feedback loop.
*   **High-Fidelity Actuator Trade-Off:** 
    *   Extended the model to evaluate system robustness under environmental disturbances (aerodynamic drag, gravity gradient) and navigation sensor errors (Star Tracker and Gyroscope noise).
    *   Conducted a comparative performance analysis across three physical actuator architectures implemented in Simulink: Reaction Wheels (RW), Magnetorquers (MTQ) and Thrusters (THR).
    *   Results & Documentation: High-resolution plots document the dynamic behavior, power profiles, and failure modes.
    *  *Note:* The exported images present smoothed curves for visual clarity; un-decimated high-frequency sensor noise and raw dynamic states remain fully modeled within the underlying Simulink block diagrams and MATLAB scripts.

---

## 🛠 Tools & Skills
*   **Software:** Python (Orekit, NumPy, SciPy, Matplotlib), MATLAB, Simulink (MATLAB Functions, Block Diagrams), GMAT (General Mission Analysis Tool).
*   **Core Competencies:** Astrodynamics, Orbital Perturbations (J2, Drag, SRP, 3rd-Body), Trajectory Optimization, Station Keeping, Attitude Determination and Control (ADCS/GNC), Actuator Trade-Off Analysis, Orbit Determination (Extended Kalman Filter), Sensor Fusion (GNSS PVT), Numerical Targeting.

---
*Note: Plot images, 3D trajectory visualizations, and simulation scripts are located within their respective sub-folders.*
