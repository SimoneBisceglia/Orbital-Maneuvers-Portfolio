# Orbital-Maneuvers-Portfolio
Orbital Maneuvers & Mission Analysis Portfolio
Developed GMAT and MATLAB simulations for advanced flight dynamics missions, focusing on trajectory optimization, Delta-V calculation, and perturbed environments:

In-Plane Maneuvers:

Hohmann vs. Bi-Elliptic Transfers: Simulated radius change maneuvers. Demonstrated that for large target-to-initial radius ratios, the Bi-Elliptic transfer is more fuel-efficient than Hohmann by exploiting lower velocities at higher apogees.

Out-of-Plane Maneuvers & Perturbations:

Inclination Changes (1-Impulse vs. 3-Impulse): Evaluated different plane change strategies, proving the energy efficiency of shifting the node to the apogee (3-Impulse) compared to a Single-Impulse burn.

General vs. Restricted Optimization: Demonstrated that the General 3-Impulse method yields the absolute global minimum Delta-V by optimizing thrust angles across all maneuver nodes.

J2 Perturbation & Target Compensation: Modeled the Earth's oblateness (J2 effect) causing RAAN (Right Ascension of the Ascending Node) regression, implementing time-of-flight drift compensation to ensure precise target orbit insertion.

Earth-Moon-Earth Mission:

Lunar Transfer (Patched Conics): Modeled the full trajectory architecture, computing escape (TLI) and capture (LOI) maneuvers for lunar orbit insertion.

Atmospheric Re-entry & Drag: Simulated a highly energetic return trajectory (11 km/s at 122 km interface), implementing atmospheric drag models to compute the capture dynamics.

Numerical Targeting: Solved complex convergence issues in GMAT using a 2-DOF strategy (varying Velocity and Normal burn components) to precisely hit the narrow re-entry corridor.

Tools & Skills: MATLAB, GMAT, Astrodynamics, Orbital Perturbations, Trajectory Optimization, Delta-V Budgeting.
